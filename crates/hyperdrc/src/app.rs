use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow};
use csgrs::io::gerber::FromGerber;

use crate::cli::{Check, Cli, DEFAULT_CHECKS, OutputFormat};
use crate::config::{self, EffectiveRules};
use crate::gerber_metadata::{
    GerberApertureDefinition, GerberApertureMacro, GerberApertureUse, GerberAttributeDelete,
    GerberCoordinateFormat, GerberCoordinateOperation, GerberImageSetup, GerberImageTransform,
    GerberInterpolationEvent, GerberLayerMetadata, GerberMetadataIssue, GerberObjectMetadata,
    GerberPolarityChange, GerberQuadrantEvent, GerberRegionEvent, GerberStepRepeatEvent,
    GerberUnits, parse_gerber_metadata_report,
};
use crate::io::{self, SourceRecord};
use crate::report::{Diagnostic, Report, Severity, Violation, report_summary, report_to_geojson};
use crate::{LayerMetadata, PcbSketch};
use crate::{
    baseline, checks, conversion, excellon, github_annotations, html_report, ipc356, jsonl, junit,
    kicad, sarif, svg_overlay, waiver,
};

const LOCAL_COPPER_DENSITY_WINDOW_MULTIPLIER: f64 = 100.0;

#[derive(Clone, Debug)]
struct Layer {
    path: PathBuf,
    source: SourceRecord,
    gerber_image_setup: GerberImageSetup,
    gerber_metadata: GerberLayerMetadata,
    gerber_aperture_definitions: Vec<GerberApertureDefinition>,
    gerber_aperture_macros: Vec<GerberApertureMacro>,
    gerber_aperture_uses: Vec<GerberApertureUse>,
    gerber_coordinate_operations: Vec<GerberCoordinateOperation>,
    gerber_polarity_changes: Vec<GerberPolarityChange>,
    gerber_image_transforms: Vec<GerberImageTransform>,
    gerber_region_events: Vec<GerberRegionEvent>,
    gerber_step_repeat_events: Vec<GerberStepRepeatEvent>,
    gerber_interpolation_events: Vec<GerberInterpolationEvent>,
    gerber_quadrant_events: Vec<GerberQuadrantEvent>,
    gerber_object_metadata: Vec<GerberObjectMetadata>,
    gerber_attribute_deletes: Vec<GerberAttributeDelete>,
    gerber_metadata_issues: Vec<GerberMetadataIssue>,
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
    manufacturing_handoff_files: Vec<io::DiscoveredFile>,
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
                    min_solder_mask_opening_area_ratio: cli.min_solder_mask_opening_area_ratio,
                    max_solder_mask_opening_area_ratio: cli.max_solder_mask_opening_area_ratio,
                    stencil_thickness: cli.stencil_thickness,
                    min_stencil_area_ratio: cli.min_stencil_area_ratio,
                    min_width: cli.min_width,
                    min_mask_width: cli.min_mask_width,
                    min_solder_mask_annular_ring: cli.min_solder_mask_annular_ring,
                    min_silkscreen_text_height: cli.min_silkscreen_text_height,
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
                && cli.package_archives.is_empty()
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
                    "provide at least one Gerber file, --gerber-dir, --package-archive, --convert-input, --kicad-pcb, --excellon, --ipc356, --bom, --centroid, --netlist, --fab-drawing, --assembly-drawing, --readme, or --rout-drawing input"
                ));
            }
            Ok(())
        })?;

        let extracted_packages = status_activity("extract package archives", || {
            crate::package_archive::ExtractedPackages::extract(&cli.package_archives)
        })?;
        let conversion_outputs =
            status_activity("run input conversions", || run_conversions(&cli))?;
        let layers = status_activity("load Gerber layers", || {
            load_all_layers(
                &cli.files,
                &cli.gerber_dirs,
                extracted_packages.packages(),
                &conversion_outputs,
            )
        })?;
        let cli = status_activity("infer Gerber layer roles", || {
            Ok(cli_with_inferred_layer_roles(cli, &layers))
        })?;
        let mut boards = status_activity("load KiCad boards", || load_boards(&cli.kicad_pcbs))?;
        let discovered_sidecars = status_activity("discover package sidecars", || {
            let mut sidecars = io::discover_package_sidecars(&cli.gerber_dirs)?;
            let archive_packages = extracted_packages
                .packages()
                .iter()
                .map(|package| (package.archive.clone(), package.directory.clone()))
                .collect::<Vec<_>>();
            extend_package_sidecars(
                &mut sidecars,
                io::discover_package_sidecars_from_archives(&archive_packages)?,
            );
            let converted_packages = conversion_outputs
                .iter()
                .map(|output| (output.source_dir.clone(), output.gerber_dir.clone()))
                .collect::<Vec<_>>();
            extend_package_sidecars(
                &mut sidecars,
                io::discover_package_sidecars_from_conversions(&converted_packages)?,
            );
            Ok(sidecars)
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
            if waiver_governance_selected(&checks) {
                violations.extend(waiver::governance_violations(&waivers));
            }
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
                    .chain(cli.package_archives.iter())
                    .chain(cli.conversion_inputs.iter())
                    .chain(cli.kicad_pcbs.iter())
                    .chain(cli.waiver_files.iter())
                    .map(|path| path.display().to_string())
                    .chain(package_input_paths_flat(&package_inputs))
                    .collect(),
                inputs: input_manifest(&cli, &layers, &package_inputs),
                diagnostics: parser_diagnostics(
                    &layers,
                    &excellon_reports,
                    &ipc356_reports,
                    &conversion_outputs,
                    &package_inputs,
                ),
                violation_count: violations.len(),
                waived_count: waived.len(),
                waived_violations: waived,
                summary,
                violations,
            })
        })?;

        if let Some(sqlite_report) = &cli.sqlite_report {
            status_activity("write SQLite report", || {
                crate::sqlite_report::write_report_sqlite(&report, sqlite_report)
            })?;
        }
        if let Some(arrow_report) = &cli.arrow_report {
            status_activity("write Arrow report", || {
                crate::arrow_report::write_report_arrow(&report, arrow_report)
            })?;
        }
        if let Some(parquet_report) = &cli.parquet_report {
            status_activity("write Parquet report", || {
                crate::parquet_report::write_report_parquet(&report, parquet_report)
            })?;
        }
        if let Some(svg_overlay) = &cli.svg_overlay {
            status_activity("write SVG overlay", || {
                std::fs::write(svg_overlay, svg_overlay::report_to_svg(&report))
                    .with_context(|| format!("failed to write {}", svg_overlay.display()))
            })?;
        }
        if let Some(gerber_overlay) = &cli.gerber_overlay {
            status_activity("write Gerber overlay", || {
                std::fs::write(
                    gerber_overlay,
                    crate::gerber_overlay::report_to_gerber(&report),
                )
                .with_context(|| format!("failed to write {}", gerber_overlay.display()))
            })?;
        }
        if let Some(gerber_keepout_overlay) = &cli.gerber_keepout_overlay {
            status_activity("write Gerber keepout overlay", || {
                std::fs::write(
                    gerber_keepout_overlay,
                    crate::gerber_overlay::report_to_gerber_keepout(&report),
                )
                .with_context(|| format!("failed to write {}", gerber_keepout_overlay.display()))
            })?;
        }
        if let Some(excellon_overlay) = &cli.excellon_overlay {
            status_activity("write Excellon overlay", || {
                std::fs::write(
                    excellon_overlay,
                    crate::excellon_overlay::report_to_excellon(&report),
                )
                .with_context(|| format!("failed to write {}", excellon_overlay.display()))
            })?;
        }
        if let Some(dxf_overlay) = &cli.dxf_overlay {
            status_activity("write DXF overlay", || {
                std::fs::write(dxf_overlay, crate::dxf_overlay::report_to_dxf(&report))
                    .with_context(|| format!("failed to write {}", dxf_overlay.display()))
            })?;
        }
        if let Some(pdf_overlay) = &cli.pdf_overlay {
            status_activity("write PDF overlay", || {
                std::fs::write(pdf_overlay, crate::pdf_overlay::report_to_pdf(&report))
                    .with_context(|| format!("failed to write {}", pdf_overlay.display()))
            })?;
        }
        if let Some(kicad_dru_output) = &cli.kicad_dru_output {
            status_activity("write KiCad custom rules", || {
                std::fs::write(
                    kicad_dru_output,
                    crate::kicad_dru::rules_to_kicad_dru(&rules),
                )
                .with_context(|| format!("failed to write {}", kicad_dru_output.display()))
            })?;
        }
        if let Some(kicad_dru_merge_output) = &cli.kicad_dru_merge_output {
            status_activity("write KiCad merged custom rules", || {
                let input = cli.kicad_dru_merge_input.as_ref().ok_or_else(|| {
                    anyhow!("--kicad-dru-merge-output requires --kicad-dru-merge-input")
                })?;
                let existing = std::fs::read_to_string(input)
                    .with_context(|| format!("failed to read {}", input.display()))?;
                std::fs::write(
                    kicad_dru_merge_output,
                    crate::kicad_dru::merge_rules_into_kicad_dru(&existing, &rules),
                )
                .with_context(|| format!("failed to write {}", kicad_dru_merge_output.display()))
            })?;
        }
        if let Some(kicad_marker_output) = &cli.kicad_marker_output {
            status_activity("write KiCad marker board", || {
                std::fs::write(
                    kicad_marker_output,
                    crate::kicad_markers::report_to_kicad_markers(&report),
                )
                .with_context(|| format!("failed to write {}", kicad_marker_output.display()))
            })?;
        }
        if let Some(kicad_marker_merge_output) = &cli.kicad_marker_merge_output {
            status_activity("write KiCad marker merge board", || {
                let source_board = cli.kicad_pcbs.first().ok_or_else(|| {
                    anyhow!("--kicad-marker-merge-output requires at least one --kicad-pcb input")
                })?;
                let source = std::fs::read_to_string(source_board)
                    .with_context(|| format!("failed to read {}", source_board.display()))?;
                std::fs::write(
                    kicad_marker_merge_output,
                    crate::kicad_markers::merge_report_into_kicad_board(&source, &report),
                )
                .with_context(|| format!("failed to write {}", kicad_marker_merge_output.display()))
            })?;
        }
        if let Some(ipc356_review_output) = &cli.ipc356_review_output {
            status_activity("write IPC-D-356 review output", || {
                std::fs::write(
                    ipc356_review_output,
                    crate::ipc356_review::report_to_ipc356_review(&report, &ipc356_reports),
                )
                .with_context(|| format!("failed to write {}", ipc356_review_output.display()))
            })?;
        }
        if let Some(gencad_review_output) = &cli.gencad_review_output {
            status_activity("write GenCAD review output", || {
                std::fs::write(
                    gencad_review_output,
                    crate::gencad_review::report_to_gencad_review(&report, &ipc356_reports),
                )
                .with_context(|| format!("failed to write {}", gencad_review_output.display()))
            })?;
        }
        if let Some(ipc2581_review_output) = &cli.ipc2581_review_output {
            status_activity("write IPC-2581 review output", || {
                std::fs::write(
                    ipc2581_review_output,
                    crate::ipc2581_review::report_to_ipc2581_review(&report),
                )
                .with_context(|| format!("failed to write {}", ipc2581_review_output.display()))
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
/// converts active findings into the process exit status expected by CI unless
/// the caller explicitly requested report-only behavior with
/// [`Cli::allow_findings`].
pub fn run_cli(cli: Cli) -> Result<()> {
    crate::process_lifecycle::install_cli_termination_handler()
        .context("failed to install CLI termination handler")?;
    let allow_findings = cli.allow_findings;
    let outcome = run(cli)?;
    if should_fail_on_findings(allow_findings, outcome.report.violation_count) {
        std::process::exit(1);
    }
    Ok(())
}

fn should_fail_on_findings(allow_findings: bool, violation_count: usize) -> bool {
    !allow_findings && violation_count > 0
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
                    for layer_index in selected_layers(layers.len(), &cli.mask_layers) {
                        let layer = &layers[layer_index];
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
                        if ipc356_points.is_empty() {
                            violations.extend(checks::copper_overlap(
                                &layer_name(left_layer),
                                &left_layer.sketch,
                                &layer_name(right_layer),
                                &right_layer.sketch,
                                rules.min_area,
                            ));
                        } else {
                            violations.extend(checks::copper_overlap_with_ipc356(
                                &layer_name(left_layer),
                                &left_layer.sketch,
                                &layer_name(right_layer),
                                &right_layer.sketch,
                                ipc356_points,
                                rules.min_area,
                            ));
                        }
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
                Check::SolderMaskOpeningRatioReadiness => {
                    for (copper_index, mask_index) in
                        explicit_layer_pairs(layers.len(), &cli.mask_pairs)?
                    {
                        let copper = &layers[copper_index];
                        let mask = &layers[mask_index];
                        violations.extend(checks::solder_mask_opening_ratio_readiness(
                            &layer_name(copper),
                            &copper.sketch,
                            &layer_name(mask),
                            &mask.sketch,
                            rules.min_solder_mask_opening_area_ratio,
                            rules.max_solder_mask_opening_area_ratio,
                            rules.min_area,
                        ));
                    }
                }
                Check::SolderMaskAnnularRingReadiness => {
                    for (copper_index, mask_index) in
                        explicit_layer_pairs(layers.len(), &cli.mask_pairs)?
                    {
                        let copper = &layers[copper_index];
                        let mask = &layers[mask_index];
                        violations.extend(checks::solder_mask_annular_ring_readiness(
                            &layer_name(copper),
                            &copper.sketch,
                            &layer_name(mask),
                            &mask.sketch,
                            rules.min_solder_mask_annular_ring,
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
                Check::SilkscreenTextHeightReadiness => {
                    let silk_layers = selected_layers(layers.len(), &cli.silk_layers);
                    let item_count = silk_layers.len();
                    for (item_index, silk_index) in silk_layers.into_iter().enumerate() {
                        let silk = &layers[silk_index];
                        let name = layer_name(silk);
                        progress_check_item(
                            check_name,
                            item_index,
                            item_count,
                            &format!("checking text height on silk layer {name}"),
                            check_started,
                        );
                        violations.extend(checks::silkscreen_text_height_readiness(
                            &name,
                            &silk.sketch,
                            rules.min_silkscreen_text_height,
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
                Check::AcidTrapTraceJunction => {
                    for board in boards {
                        violations.extend(checks::trace_junction_acid_trap_readiness(
                            board,
                            kicad_copper_layers,
                            rules.acid_trap_angle,
                            rules.min_area,
                        ));
                    }
                }
                Check::LayerSanity => {
                    for layer in layers {
                        violations.extend(checks::layer_sanity(
                            &layer_name(layer),
                            &layer.sketch,
                            rules.max_layer_area,
                        ));
                        violations.extend(checks::tiny_layer_feature_readiness(
                            &layer_name(layer),
                            &layer.sketch,
                            rules.min_area,
                        ));
                        violations.extend(checks::skinny_layer_feature_readiness(
                            &layer_name(layer),
                            &layer.sketch,
                            rules.min_width,
                            rules.min_area,
                        ));
                        violations.extend(checks::duplicate_layer_island_readiness(
                            &layer_name(layer),
                            &layer.sketch,
                            rules.min_area,
                        ));
                    }
                    let explicit_layers = layers
                        .iter()
                        .map(|layer| (layer_name(layer), layer.sketch.clone()))
                        .collect::<Vec<_>>();
                    violations.extend(checks::duplicate_layer_geometry_readiness(
                        &explicit_layers,
                        rules.min_area,
                    ));
                    for board in boards {
                        for (layer_name, copper) in board.copper_layers(kicad_copper_layers) {
                            let name = format!("{}:{layer_name}", board.source);
                            violations.extend(checks::layer_sanity(
                                &name,
                                &copper,
                                rules.max_layer_area,
                            ));
                            violations.extend(checks::tiny_layer_feature_readiness(
                                &name,
                                &copper,
                                rules.min_area,
                            ));
                            violations.extend(checks::skinny_layer_feature_readiness(
                                &name,
                                &copper,
                                rules.min_width,
                                rules.min_area,
                            ));
                            violations.extend(checks::duplicate_layer_island_readiness(
                                &name,
                                &copper,
                                rules.min_area,
                            ));
                        }
                        let kicad_layers = board
                            .copper_layers(kicad_copper_layers)
                            .into_iter()
                            .map(|(layer_name, copper)| {
                                (format!("{}:{layer_name}", board.source), copper)
                            })
                            .collect::<Vec<_>>();
                        violations.extend(checks::duplicate_layer_geometry_readiness(
                            &kicad_layers,
                            rules.min_area,
                        ));
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
                Check::LocalCopperDensityReadiness => {
                    if !cli.copper_layers.is_empty() {
                        let gerber_copper = cli
                            .copper_layers
                            .iter()
                            .map(|index| {
                                let layer = &layers[*index];
                                (layer_name(layer), layer.sketch.clone())
                            })
                            .collect::<Vec<_>>();
                        violations.extend(checks::local_copper_density_readiness(
                            &gerber_copper,
                            local_copper_density_window(rules.min_width),
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
                        violations.extend(checks::local_copper_density_readiness(
                            &kicad_copper,
                            local_copper_density_window(rules.min_width),
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
                Check::DrillCopperClearance | Check::DrillToCopperClearance => {
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
                Check::DifferentialPairWidthReadiness => {
                    for board in boards {
                        violations.extend(checks::differential_pair_width_readiness(
                            board,
                            kicad_copper_layers,
                            rules.min_width,
                            rules.min_width * 0.5,
                        ));
                    }
                }
                Check::DifferentialPairNeckdownReadiness => {
                    for board in boards {
                        violations.extend(checks::differential_pair_neckdown_readiness(
                            board,
                            kicad_copper_layers,
                            rules.min_width,
                            rules.net_clearance * 8.0,
                        ));
                    }
                }
                Check::DifferentialPairSkewReadiness => {
                    for board in boards {
                        violations.extend(checks::differential_pair_skew_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 4.0,
                        ));
                    }
                }
                Check::DifferentialPairToPairSpacingReadiness => {
                    for board in boards {
                        violations.extend(checks::differential_pair_to_pair_spacing_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 6.0,
                        ));
                    }
                }
                Check::DifferentialPairViaProximityReadiness => {
                    for board in boards {
                        violations.extend(checks::differential_pair_via_proximity_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 4.0,
                        ));
                    }
                }
                Check::DifferentialPairViaReturnReadiness => {
                    for board in boards {
                        violations.extend(checks::differential_pair_via_return_readiness(
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
                Check::SplitPlaneCrossingReadiness => {
                    for board in boards {
                        violations.extend(checks::split_plane_crossing_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance,
                            rules.min_area,
                        ));
                    }
                }
                Check::ReturnPathProximityReadiness => {
                    for board in boards {
                        violations.extend(checks::return_path_proximity_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 2.0,
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
                Check::SameNetDrillBreakReadiness => {
                    for board in boards {
                        violations.extend(checks::same_net_drill_break_readiness(
                            board,
                            excellon_drills,
                            kicad_copper_layers,
                            rules.min_area,
                        ));
                    }
                }
                Check::DifferentNetShortReadiness => {
                    for board in boards {
                        violations.extend(checks::different_net_short_readiness(
                            board,
                            kicad_copper_layers,
                            rules.min_area,
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
                Check::PowerViaReturnReadiness => {
                    for board in boards {
                        violations.extend(checks::power_via_return_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 2.0,
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
                Check::ThermalViaDistributionReadiness => {
                    for board in boards {
                        violations.extend(checks::thermal_via_distribution_readiness(
                            board,
                            kicad_copper_layers,
                            2,
                            rules.net_clearance * 4.0,
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
                Check::PowerPadEntryReadiness => {
                    for board in boards {
                        violations.extend(checks::power_pad_entry_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance,
                            rules.min_width * 2.0,
                            2,
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
                Check::ProtectiveEarthSpacingReadiness => {
                    for board in boards {
                        violations.extend(checks::protective_earth_spacing_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 3.0,
                            rules.min_area,
                        ));
                    }
                }
                Check::SurgeProtectionKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::surge_protection_keepout_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 2.0,
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
                Check::MixedSignalPartitionReadiness => {
                    for board in boards {
                        violations.extend(checks::mixed_signal_partition_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 3.0,
                            rules.net_clearance * 2.0,
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
                Check::AntennaCopperKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::antenna_copper_keepout_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 6.0,
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
                Check::EsdReturnPathReadiness => {
                    for board in boards {
                        violations.extend(checks::esd_return_path_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 4.0,
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
                Check::InductorCopperKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::inductor_copper_keepout_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 6.0,
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
                Check::DensePadViaSpacingReadiness => {
                    for board in boards {
                        violations.extend(checks::dense_pad_via_spacing_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.dense_pad_pitch,
                            rules.assembly.dense_pad_via_search_radius,
                            rules.net_clearance,
                            rules.min_area,
                        ));
                    }
                }
                Check::DensePadMaskBridgeReadiness => {
                    for board in boards {
                        violations.extend(checks::dense_pad_mask_bridge_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.dense_pad_pitch,
                            rules.min_mask_width,
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
                Check::NetSpacing | Check::DifferentNetSpacing => {
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
                Check::RegistrationTolerance | Check::LayerRegistrationTolerance => {
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
                    let bom_files = load_text_artifacts_with_sheets(
                        &package_input_paths(&package_inputs.bom_files),
                        &cli.bom_sheet_names,
                    )?;
                    let centroid_files = load_text_artifacts_with_sheets(
                        &package_input_paths(&package_inputs.centroid_files),
                        &cli.centroid_sheet_names,
                    )?;
                    let netlist_files = load_text_artifacts_with_sheets(
                        &package_input_paths(&package_inputs.netlist_files),
                        &cli.netlist_sheet_names,
                    )?;
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
                Check::WaiverGovernance => {
                    // Waiver metadata must be audited after normal findings are
                    // waived, otherwise a waiver file could suppress warnings
                    // about itself. The selectable check is recorded here for
                    // progress output; findings are appended in `run`.
                    log::trace!("waiver-governance readiness deferred until after waiver matching");
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
        Check::SolderMaskOpeningRatioReadiness => "solder-mask-opening-ratio-readiness",
        Check::SolderMaskAnnularRingReadiness => "solder-mask-annular-ring-readiness",
        Check::SolderMaskExpansion => "solder-mask-expansion",
        Check::SolderMaskOverlapClearance => "solder-mask-overlap-clearance",
        Check::SolderMaskBoardEdgeClearance => "solder-mask-board-edge-clearance",
        Check::SilkscreenOverlap => "silkscreen-overlap",
        Check::SilkscreenClearance => "silkscreen-clearance",
        Check::SilkscreenBoardEdgeClearance => "silkscreen-board-edge-clearance",
        Check::SilkscreenMinWidth => "silkscreen-min-width",
        Check::SilkscreenTextHeightReadiness => "silkscreen-text-height-readiness",
        Check::MinCopperNeck => "min-copper-neck",
        Check::AcidTrap => "acid-trap",
        Check::AcidTrapTraceJunction => "acid-trap-trace-junction",
        Check::LayerSanity => "layer-sanity",
        Check::CopperBalance => "copper-balance",
        Check::LocalCopperDensityReadiness => "local-copper-density-readiness",
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
        Check::DrillToCopperClearance => "drill-to-copper-clearance",
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
        Check::DifferentialPairWidthReadiness => "differential-pair-width-readiness",
        Check::DifferentialPairNeckdownReadiness => "differential-pair-neckdown-readiness",
        Check::DifferentialPairSkewReadiness => "differential-pair-skew-readiness",
        Check::DifferentialPairToPairSpacingReadiness => {
            "differential-pair-to-pair-spacing-readiness"
        }
        Check::DifferentialPairViaProximityReadiness => "differential-pair-via-proximity-readiness",
        Check::DifferentialPairViaReturnReadiness => "differential-pair-via-return-readiness",
        Check::DifferentialPairViaSymmetryReadiness => "differential-pair-via-symmetry-readiness",
        Check::DifferentialPairReturnReadiness => "differential-pair-return-readiness",
        Check::ReferencePlaneReadiness => "reference-plane-readiness",
        Check::ReferencePlaneVoidReadiness => "reference-plane-void-readiness",
        Check::SplitPlaneCrossingReadiness => "split-plane-crossing-readiness",
        Check::ReturnPathProximityReadiness => "return-path-proximity-readiness",
        Check::OrphanedZoneReadiness => "orphaned-zone-readiness",
        Check::SameNetIslandReadiness => "same-net-island-readiness",
        Check::SameNetDrillBreakReadiness => "same-net-drill-break-readiness",
        Check::DifferentNetShortReadiness => "different-net-short-readiness",
        Check::ReturnPathReadiness => "return-path-readiness",
        Check::HighCurrentReadiness => "high-current-readiness",
        Check::PowerViaArrayReadiness => "power-via-array-readiness",
        Check::PowerViaReturnReadiness => "power-via-return-readiness",
        Check::ThermalViaReadiness => "thermal-via-readiness",
        Check::ThermalViaDistributionReadiness => "thermal-via-distribution-readiness",
        Check::PowerPlaneReadiness => "power-plane-readiness",
        Check::HighCurrentNeckReadiness => "high-current-neck-readiness",
        Check::PowerPadEntryReadiness => "power-pad-entry-readiness",
        Check::VoltageClearanceReadiness => "voltage-clearance-readiness",
        Check::ProtectiveEarthSpacingReadiness => "protective-earth-spacing-readiness",
        Check::SurgeProtectionKeepoutReadiness => "surge-protection-keepout-readiness",
        Check::SensitiveNetSpacingReadiness => "sensitive-net-spacing-readiness",
        Check::SensitiveReturnReadiness => "sensitive-return-readiness",
        Check::MixedSignalPartitionReadiness => "mixed-signal-partition-readiness",
        Check::RfKeepoutReadiness => "rf-keepout-readiness",
        Check::AntennaCopperKeepoutReadiness => "antenna-copper-keepout-readiness",
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
        Check::EsdReturnPathReadiness => "esd-return-path-readiness",
        Check::SwitchNodeKeepoutReadiness => "switch-node-keepout-readiness",
        Check::InductorCopperKeepoutReadiness => "inductor-copper-keepout-readiness",
        Check::TestpointCoverageReadiness => "testpoint-coverage-readiness",
        Check::TestpointAccessibilityReadiness => "testpoint-accessibility-readiness",
        Check::TestpointCopperClearanceReadiness => "testpoint-copper-clearance-readiness",
        Check::ToolingHoleReadiness => "tooling-hole-readiness",
        Check::MouseBiteReadiness => "mouse-bite-readiness",
        Check::FiducialReadiness => "fiducial-readiness",
        Check::LocalFiducialReadiness => "local-fiducial-readiness",
        Check::FiducialKeepoutReadiness => "fiducial-keepout-readiness",
        Check::DensePadEscapeReadiness => "dense-pad-escape-readiness",
        Check::DensePadViaSpacingReadiness => "dense-pad-via-spacing-readiness",
        Check::DensePadMaskBridgeReadiness => "dense-pad-mask-bridge-readiness",
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
        Check::DifferentNetSpacing => "different-net-spacing",
        Check::RegistrationTolerance => "registration-tolerance",
        Check::LayerRegistrationTolerance => "layer-registration-tolerance",
        Check::PanelizationClearance => "panelization-clearance",
        Check::Ipc356Coverage => "ipc356-coverage",
        Check::Ipc356DrillDiameter => "ipc356-drill-diameter",
        Check::ExcellonReadiness => "excellon-readiness",
        Check::FileManifestReadiness => "file-manifest-readiness",
        Check::ProductionArtifactReadiness => "production-artifact-readiness",
        Check::StackupReadiness => "stackup-readiness",
        Check::NetConstraintReadiness => "net-constraint-readiness",
        Check::WaiverGovernance => "waiver-governance",
    }
}

fn waiver_governance_selected(checks: &[Check]) -> bool {
    checks.contains(&Check::WaiverGovernance)
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
    extend_package_inputs(
        &mut inputs.manufacturing_handoff_files,
        discovered.manufacturing_handoff_files,
    );
    inputs
}

fn extend_package_sidecars(target: &mut io::PackageSidecars, source: io::PackageSidecars) {
    target.excellon_files.extend(source.excellon_files);
    target.ipc356_files.extend(source.ipc356_files);
    target.bom_files.extend(source.bom_files);
    target.centroid_files.extend(source.centroid_files);
    target.netlist_files.extend(source.netlist_files);
    target.fab_drawing_files.extend(source.fab_drawing_files);
    target
        .assembly_drawing_files
        .extend(source.assembly_drawing_files);
    target.readme_files.extend(source.readme_files);
    target.rout_drawing_files.extend(source.rout_drawing_files);
    target
        .manufacturing_handoff_files
        .extend(source.manufacturing_handoff_files);
    target.sort();
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
        .chain(inputs.manufacturing_handoff_files.iter())
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
        .chain(inputs.manufacturing_handoff_files.iter())
        .map(|input| input.source.clone())
}

fn load_text_artifacts(files: &[PathBuf]) -> Result<Vec<checks::TextArtifact>> {
    load_text_artifacts_with_sheets(files, &[])
}

fn load_text_artifacts_with_sheets(
    files: &[PathBuf],
    spreadsheet_sheet_names: &[String],
) -> Result<Vec<checks::TextArtifact>> {
    files
        .iter()
        .map(|path| {
            let text = load_text_artifact_with_sheets(path, spreadsheet_sheet_names)?;
            Ok(checks::TextArtifact {
                path: path.display().to_string(),
                text,
            })
        })
        .collect()
}

fn load_text_artifact(path: &std::path::Path) -> Result<String> {
    load_text_artifact_with_sheets(path, &[])
}

fn load_text_artifact_with_sheets(
    path: &std::path::Path,
    spreadsheet_sheet_names: &[String],
) -> Result<String> {
    let bytes =
        std::fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    if is_spreadsheet_path(path) {
        match load_spreadsheet_text_with_sheets(path, spreadsheet_sheet_names) {
            Ok(text) => return Ok(text),
            Err(error) if !spreadsheet_sheet_names.is_empty() => return Err(error),
            Err(_) => {}
        }
    }
    if is_json_path(path) {
        if let Ok(text) = load_json_table_text(&bytes) {
            return Ok(text);
        }
    }
    Ok(match String::from_utf8(bytes) {
        Ok(text) => text,
        Err(error) => String::from_utf8_lossy(error.as_bytes()).into_owned(),
    })
}

fn is_spreadsheet_path(path: &std::path::Path) -> bool {
    path.extension()
        .and_then(|extension| extension.to_str())
        .map(|extension| extension.to_ascii_lowercase())
        .is_some_and(|extension| {
            matches!(extension.as_str(), "xls" | "xlsx" | "xlsm" | "xlsb" | "ods")
        })
}

fn is_json_path(path: &std::path::Path) -> bool {
    path.extension()
        .and_then(|extension| extension.to_str())
        .is_some_and(|extension| extension.eq_ignore_ascii_case("json"))
}

fn load_json_table_text(bytes: &[u8]) -> Result<String> {
    let value: serde_json::Value = serde_json::from_slice(bytes)?;
    json_value_to_table_text(&value).ok_or_else(|| anyhow!("JSON sidecar is not table-shaped"))
}

fn json_value_to_table_text(value: &serde_json::Value) -> Option<String> {
    match value {
        serde_json::Value::Array(rows) => json_array_to_table_text(rows),
        serde_json::Value::Object(object) => {
            if let Some(rows) = find_json_table_array(object) {
                return json_array_to_table_text(rows);
            }
            let rows = [object];
            json_object_rows_to_table_text(&rows)
        }
        _ => None,
    }
}

fn find_json_table_array(
    object: &serde_json::Map<String, serde_json::Value>,
) -> Option<&[serde_json::Value]> {
    for key in [
        "bom",
        "bill_of_materials",
        "components",
        "parts",
        "centroid",
        "placements",
        "positions",
        "netlist",
        "nets",
        "records",
        "rows",
        "items",
        "data",
        "results",
    ] {
        if let Some(value) = object.get(key) {
            match value {
                serde_json::Value::Array(rows) if json_array_is_table_like(rows) => {
                    return Some(rows);
                }
                serde_json::Value::Object(child) => {
                    if let Some(rows) = find_json_table_array(child) {
                        return Some(rows);
                    }
                }
                _ => {}
            }
        }
    }
    for value in object.values() {
        if let serde_json::Value::Object(child) = value
            && let Some(rows) = find_json_table_array(child)
        {
            return Some(rows);
        }
    }
    None
}

fn json_array_is_table_like(rows: &[serde_json::Value]) -> bool {
    rows.is_empty()
        || rows.iter().all(serde_json::Value::is_object)
        || rows.iter().all(serde_json::Value::is_array)
}

fn json_array_to_table_text(rows: &[serde_json::Value]) -> Option<String> {
    if rows.is_empty() {
        return Some(String::new());
    }
    if rows.iter().all(serde_json::Value::is_object) {
        let objects = rows
            .iter()
            .filter_map(serde_json::Value::as_object)
            .collect::<Vec<_>>();
        return json_object_rows_to_table_text(&objects);
    }
    if rows.iter().all(serde_json::Value::is_array) {
        let mut text = String::new();
        for row in rows.iter().filter_map(serde_json::Value::as_array) {
            let cells = row.iter().map(json_cell_text).collect::<Vec<_>>();
            text.push_str(&cells.join("\t"));
            text.push('\n');
        }
        return Some(text);
    }
    None
}

fn json_object_rows_to_table_text(
    rows: &[&serde_json::Map<String, serde_json::Value>],
) -> Option<String> {
    let mut headers = Vec::<String>::new();
    let flattened_rows = rows
        .iter()
        .map(|row| flatten_json_object(row))
        .collect::<Vec<_>>();
    for row in &flattened_rows {
        for (key, _) in row {
            if !headers.iter().any(|header| header == key) {
                headers.push(key.clone());
            }
        }
    }
    if headers.is_empty() {
        return None;
    }
    let mut text = String::new();
    text.push_str(&headers.join("\t"));
    text.push('\n');
    for row in &flattened_rows {
        let cells = headers
            .iter()
            .map(|header| {
                row.iter()
                    .find_map(|(key, value)| (key == header).then_some(value.clone()))
                    .unwrap_or_default()
            })
            .collect::<Vec<_>>();
        if cells.iter().any(|cell| !cell.is_empty()) {
            text.push_str(&cells.join("\t"));
            text.push('\n');
        }
    }
    Some(text)
}

fn flatten_json_object(
    object: &serde_json::Map<String, serde_json::Value>,
) -> Vec<(String, String)> {
    let mut cells = Vec::new();
    for (key, value) in object {
        flatten_json_value(key, value, &mut cells);
    }
    cells
}

fn flatten_json_value(key: &str, value: &serde_json::Value, cells: &mut Vec<(String, String)>) {
    match value {
        serde_json::Value::Object(object) => {
            if object.is_empty() {
                cells.push((key.to_string(), String::new()));
            } else {
                for (child_key, child_value) in object {
                    flatten_json_value(&format!("{key}.{child_key}"), child_value, cells);
                }
            }
        }
        _ => cells.push((key.to_string(), json_cell_text(value))),
    }
}

fn json_cell_text(value: &serde_json::Value) -> String {
    match value {
        serde_json::Value::Null => String::new(),
        serde_json::Value::String(value) => sanitize_spreadsheet_cell(value),
        serde_json::Value::Number(value) => value.to_string(),
        serde_json::Value::Bool(value) => value.to_string(),
        serde_json::Value::Array(values) => sanitize_spreadsheet_cell(
            &values
                .iter()
                .map(json_cell_text)
                .filter(|value| !value.is_empty())
                .collect::<Vec<_>>()
                .join(" "),
        ),
        serde_json::Value::Object(_) => sanitize_spreadsheet_cell(&value.to_string()),
    }
}

fn load_spreadsheet_text_with_sheets(
    path: &std::path::Path,
    sheet_names: &[String],
) -> Result<String> {
    use calamine::{Reader, open_workbook_auto};

    let mut workbook =
        open_workbook_auto(path).with_context(|| format!("failed to read {}", path.display()))?;
    let requested_sheet_names = sheet_names
        .iter()
        .map(|name| name.trim())
        .filter(|name| !name.is_empty())
        .map(|name| name.to_ascii_lowercase())
        .collect::<Vec<_>>();
    let formula_comments = workbook_formula_comments(&mut workbook, &requested_sheet_names);
    if !requested_sheet_names.is_empty() {
        let requested_rows = workbook
            .worksheets()
            .into_iter()
            .filter_map(|(sheet_name, range)| {
                requested_sheet_names
                    .iter()
                    .any(|requested| requested == &sheet_name.to_ascii_lowercase())
                    .then(|| workbook_sheet_rows(&range))
            })
            .filter(|rows| !rows.is_empty())
            .collect::<Vec<_>>();
        if requested_rows.is_empty() {
            return Err(anyhow!(
                "none of the requested workbook sheets were found with table rows in {}: {}",
                path.display(),
                sheet_names.join(", ")
            ));
        }
        return Ok(format_table_text_with_comments(
            merge_workbook_tables_by_header(requested_rows),
            &formula_comments,
        ));
    }
    let mut selected_rows: Option<Vec<Vec<String>>> = None;
    let mut selected_header: Option<Vec<String>> = None;
    for (sheet_name, range) in workbook.worksheets() {
        if !requested_sheet_names.is_empty()
            && !requested_sheet_names
                .iter()
                .any(|requested| requested == &sheet_name.to_ascii_lowercase())
        {
            continue;
        }
        let rows = workbook_sheet_rows(&range);
        if rows.is_empty() {
            continue;
        }
        let header = workbook_header_signature(&rows[0]);
        if let (Some(existing_rows), Some(existing_header)) = (&mut selected_rows, &selected_header)
        {
            if header == *existing_header {
                existing_rows.extend(rows.into_iter().skip(1));
            }
        } else {
            selected_header = Some(header);
            selected_rows = Some(rows);
        }
    }
    Ok(format_table_text_with_comments(
        selected_rows.unwrap_or_default(),
        &formula_comments,
    ))
}

fn workbook_formula_comments(
    workbook: &mut calamine::Sheets<std::io::BufReader<std::fs::File>>,
    requested_sheet_names: &[String],
) -> Vec<String> {
    use calamine::Reader as _;

    let sheet_names = workbook.sheet_names();
    let mut comments = Vec::new();
    for sheet_name in sheet_names {
        if !requested_sheet_names.is_empty()
            && !requested_sheet_names
                .iter()
                .any(|requested| requested == &sheet_name.to_ascii_lowercase())
        {
            continue;
        }
        let Ok(formulas) = workbook.worksheet_formula(&sheet_name) else {
            continue;
        };
        let (start_row, start_column) = formulas.start().unwrap_or((0, 0));
        for (row, column, formula) in formulas.used_cells() {
            let formula = formula.trim();
            if formula.is_empty() {
                continue;
            }
            comments.push(format!(
                "# hyperdrc-workbook-formula sheet={} cell={} formula={}",
                sanitize_spreadsheet_cell(&sheet_name),
                spreadsheet_cell_ref(row + start_row as usize, column + start_column as usize),
                sanitize_spreadsheet_cell(formula)
            ));
        }
    }
    comments
}

fn workbook_sheet_rows(range: &calamine::Range<calamine::Data>) -> Vec<Vec<String>> {
    range
        .rows()
        .map(|row| row.iter().map(spreadsheet_cell_text).collect::<Vec<_>>())
        .filter(|row| row.iter().any(|cell| !cell.trim().is_empty()))
        .collect()
}

fn merge_workbook_tables_by_header(tables: Vec<Vec<Vec<String>>>) -> Vec<Vec<String>> {
    let mut headers = Vec::<String>::new();
    let mut header_signatures = Vec::<String>::new();
    let mut rows = Vec::<Vec<String>>::new();

    for table in tables {
        let Some((header, data_rows)) = table.split_first() else {
            continue;
        };
        let table_header = workbook_header_signature(header);
        for (column_index, signature) in table_header.iter().enumerate() {
            if signature.is_empty()
                || header_signatures
                    .iter()
                    .any(|existing| existing == signature)
            {
                continue;
            }
            header_signatures.push(signature.clone());
            headers.push(header.get(column_index).cloned().unwrap_or_default());
            for row in &mut rows {
                row.push(String::new());
            }
        }

        for data_row in data_rows {
            let mut merged_row = vec![String::new(); headers.len()];
            for (column_index, signature) in table_header.iter().enumerate() {
                if let Some(target_column) = header_signatures
                    .iter()
                    .position(|existing| existing == signature)
                {
                    merged_row[target_column] =
                        data_row.get(column_index).cloned().unwrap_or_default();
                }
            }
            if merged_row.iter().any(|cell| !cell.trim().is_empty()) {
                rows.push(merged_row);
            }
        }
    }

    if headers.is_empty() {
        Vec::new()
    } else {
        std::iter::once(headers).chain(rows).collect()
    }
}

fn format_table_text_with_comments(rows: Vec<Vec<String>>, comments: &[String]) -> String {
    let text = rows
        .into_iter()
        .map(|row| row.join("\t"))
        .collect::<Vec<_>>()
        .join("\n");
    let mut output = String::new();
    for comment in comments {
        output.push_str(comment);
        output.push('\n');
    }
    if !text.is_empty() {
        output.push_str(&text);
        output.push('\n');
    }
    output
}

fn spreadsheet_cell_text(cell: &calamine::Data) -> String {
    match cell {
        calamine::Data::Empty => String::new(),
        calamine::Data::String(value) => sanitize_spreadsheet_cell(value),
        _ => sanitize_spreadsheet_cell(&cell.to_string()),
    }
}

fn sanitize_spreadsheet_cell(value: &str) -> String {
    value.replace(['\t', '\r', '\n'], " ").trim().to_string()
}

fn spreadsheet_cell_ref(row: usize, column: usize) -> String {
    format!("{}{}", spreadsheet_column_name(column), row + 1)
}

fn spreadsheet_column_name(mut column: usize) -> String {
    let mut name = String::new();
    loop {
        let remainder = column % 26;
        name.insert(0, (b'A' + remainder as u8) as char);
        column /= 26;
        if column == 0 {
            break;
        }
        column -= 1;
    }
    name
}

fn workbook_header_signature(row: &[String]) -> Vec<String> {
    row.iter()
        .map(|cell| workbook_header_signature_cell(cell))
        .collect()
}

fn workbook_header_signature_cell(cell: &str) -> String {
    let normalized = cell
        .chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .collect::<String>()
        .to_ascii_lowercase();
    match normalized.as_str() {
        "ref" | "refs" | "refdes" | "reference" | "references" | "designator" | "designators" => {
            "reference"
        }
        "qty" | "qnty" | "quantity" | "quantities" => "quantity",
        "mfr" | "manufacturer" | "maker" => "manufacturer",
        "mpn" | "mfgpn" | "mfrpn" | "manufacturerpartnumber" | "partnumber" => "mpn",
        "footprint" | "package" | "pkg" | "case" => "package",
        "side" | "layer" => "side",
        "rot" | "rotation" | "angle" | "orientation" => "rotation",
        "x" | "posx" | "positionx" | "centerx" | "midx" => "x",
        "y" | "posy" | "positiony" | "centery" | "midy" => "y",
        "net" | "netname" | "signal" => "net",
        "pin" | "pad" | "terminal" => "pin",
        _ => normalized.as_str(),
    }
    .to_string()
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

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum InferredGerberRole {
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

fn cli_with_inferred_layer_roles(mut cli: Cli, layers: &[Layer]) -> Cli {
    let roles = layers.iter().map(infer_gerber_role).collect::<Vec<_>>();

    if cli.board_outline.is_none() {
        let outlines = layer_indexes_with_roles(&roles, &[InferredGerberRole::Outline]);
        if outlines.len() == 1 {
            cli.board_outline = outlines.first().copied();
        }
    }
    if cli.copper_layers.is_empty() {
        cli.copper_layers = layer_indexes_with_roles(
            &roles,
            &[
                InferredGerberRole::TopCopper,
                InferredGerberRole::BottomCopper,
                InferredGerberRole::InnerCopper,
            ],
        );
    }
    if cli.mask_layers.is_empty() {
        cli.mask_layers = layer_indexes_with_roles(
            &roles,
            &[InferredGerberRole::TopMask, InferredGerberRole::BottomMask],
        );
    }
    if cli.silk_layers.is_empty() {
        cli.silk_layers = layer_indexes_with_roles(
            &roles,
            &[InferredGerberRole::TopSilk, InferredGerberRole::BottomSilk],
        );
    }
    if cli.paste_pairs.is_empty() {
        cli.paste_pairs = inferred_same_side_pairs(
            &roles,
            InferredGerberRole::TopPaste,
            InferredGerberRole::TopCopper,
        )
        .into_iter()
        .chain(inferred_same_side_pairs(
            &roles,
            InferredGerberRole::BottomPaste,
            InferredGerberRole::BottomCopper,
        ))
        .map(|(paste, copper)| format!("{paste}:{copper}"))
        .collect();
    }
    if cli.mask_pairs.is_empty() {
        cli.mask_pairs = inferred_same_side_pairs(
            &roles,
            InferredGerberRole::TopCopper,
            InferredGerberRole::TopMask,
        )
        .into_iter()
        .chain(inferred_same_side_pairs(
            &roles,
            InferredGerberRole::BottomCopper,
            InferredGerberRole::BottomMask,
        ))
        .map(|(copper, mask)| format!("{copper}:{mask}"))
        .collect();
    }
    if cli.silk_pairs.is_empty() {
        cli.silk_pairs = inferred_silkscreen_pairs(&roles)
            .into_iter()
            .map(|(silk, blocker)| format!("{silk}:{blocker}"))
            .collect();
    }

    log::trace!(
        "inferred Gerber layer roles: board_outline={:?} copper_layers={:?} mask_layers={:?} silk_layers={:?} paste_pairs={:?} mask_pairs={:?} silk_pairs={:?}",
        cli.board_outline,
        cli.copper_layers,
        cli.mask_layers,
        cli.silk_layers,
        cli.paste_pairs,
        cli.mask_pairs,
        cli.silk_pairs
    );
    cli
}

fn infer_gerber_role(layer: &Layer) -> InferredGerberRole {
    if let Some(role) = layer
        .gerber_metadata
        .file_function
        .as_deref()
        .and_then(infer_file_function_role)
    {
        return role;
    }
    infer_path_role(&layer.source.path).unwrap_or(InferredGerberRole::Other)
}

fn infer_file_function_role(file_function: &str) -> Option<InferredGerberRole> {
    let tokens = file_function
        .split([',', '.', '-', '_', ' ', '\t'])
        .map(|token| token.trim().to_ascii_lowercase())
        .filter(|token| !token.is_empty())
        .collect::<Vec<_>>();
    let first = tokens.first()?.as_str();
    match first {
        "profile" | "outline" | "rout" | "route" => Some(InferredGerberRole::Outline),
        "copper" => match inferred_side(&tokens) {
            Some("top") => Some(InferredGerberRole::TopCopper),
            Some("bottom") => Some(InferredGerberRole::BottomCopper),
            _ => Some(InferredGerberRole::InnerCopper),
        },
        "soldermask" | "mask" => match inferred_side(&tokens) {
            Some("top") => Some(InferredGerberRole::TopMask),
            Some("bottom") => Some(InferredGerberRole::BottomMask),
            _ => None,
        },
        "paste" | "solderpaste" => match inferred_side(&tokens) {
            Some("top") => Some(InferredGerberRole::TopPaste),
            Some("bottom") => Some(InferredGerberRole::BottomPaste),
            _ => None,
        },
        "legend" | "silk" | "silkscreen" => match inferred_side(&tokens) {
            Some("top") => Some(InferredGerberRole::TopSilk),
            Some("bottom") => Some(InferredGerberRole::BottomSilk),
            _ => None,
        },
        _ => None,
    }
}

fn inferred_side(tokens: &[String]) -> Option<&'static str> {
    if tokens
        .iter()
        .any(|token| matches!(token.as_str(), "top" | "f" | "front"))
    {
        Some("top")
    } else if tokens
        .iter()
        .any(|token| matches!(token.as_str(), "bottom" | "b" | "back" | "bot"))
    {
        Some("bottom")
    } else {
        None
    }
}

fn infer_path_role(path: &str) -> Option<InferredGerberRole> {
    let lower = path.to_ascii_lowercase();
    let compact = lower
        .chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .collect::<String>();
    let extension = std::path::Path::new(path)
        .extension()
        .and_then(|extension| extension.to_str())
        .map(|extension| extension.to_ascii_lowercase());
    match extension.as_deref() {
        Some("gtl") => return Some(InferredGerberRole::TopCopper),
        Some("gbl") => return Some(InferredGerberRole::BottomCopper),
        Some("gts") => return Some(InferredGerberRole::TopMask),
        Some("gbs") => return Some(InferredGerberRole::BottomMask),
        Some("gtp") => return Some(InferredGerberRole::TopPaste),
        Some("gbp") => return Some(InferredGerberRole::BottomPaste),
        Some("gto") => return Some(InferredGerberRole::TopSilk),
        Some("gbo") => return Some(InferredGerberRole::BottomSilk),
        Some("gko") | Some("gm1") | Some("gml") => return Some(InferredGerberRole::Outline),
        _ => {}
    }
    if lower.contains("edge.cuts")
        || compact.contains("boardoutline")
        || compact.contains("fabricationoutline")
        || compact.contains("profile")
        || compact.contains("outline")
    {
        Some(InferredGerberRole::Outline)
    } else if compact.contains("topcopper") || lower.contains("f.cu") {
        Some(InferredGerberRole::TopCopper)
    } else if compact.contains("bottomcopper") || lower.contains("b.cu") {
        Some(InferredGerberRole::BottomCopper)
    } else if compact.contains("topsoldermask")
        || compact.contains("topmask")
        || lower.contains("f.mask")
    {
        Some(InferredGerberRole::TopMask)
    } else if compact.contains("bottomsoldermask")
        || compact.contains("bottommask")
        || lower.contains("b.mask")
    {
        Some(InferredGerberRole::BottomMask)
    } else if compact.contains("topsolderpaste")
        || compact.contains("toppaste")
        || lower.contains("f.paste")
    {
        Some(InferredGerberRole::TopPaste)
    } else if compact.contains("bottomsolderpaste")
        || compact.contains("bottompaste")
        || lower.contains("b.paste")
    {
        Some(InferredGerberRole::BottomPaste)
    } else if compact.contains("topsilkscreen")
        || compact.contains("topsilk")
        || lower.contains("f.silk")
    {
        Some(InferredGerberRole::TopSilk)
    } else if compact.contains("bottomsilkscreen")
        || compact.contains("bottomsilk")
        || lower.contains("b.silk")
    {
        Some(InferredGerberRole::BottomSilk)
    } else {
        None
    }
}

fn layer_indexes_with_roles(
    roles: &[InferredGerberRole],
    selected: &[InferredGerberRole],
) -> Vec<usize> {
    roles
        .iter()
        .enumerate()
        .filter_map(|(index, role)| selected.contains(role).then_some(index))
        .collect()
}

fn inferred_same_side_pairs(
    roles: &[InferredGerberRole],
    left: InferredGerberRole,
    right: InferredGerberRole,
) -> Vec<(usize, usize)> {
    let left_indexes = layer_indexes_with_roles(roles, &[left]);
    let right_indexes = layer_indexes_with_roles(roles, &[right]);
    if left_indexes.len() == 1 && right_indexes.len() == 1 {
        vec![(left_indexes[0], right_indexes[0])]
    } else {
        Vec::new()
    }
}

fn inferred_silkscreen_pairs(roles: &[InferredGerberRole]) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    pairs.extend(inferred_same_side_pairs(
        roles,
        InferredGerberRole::TopSilk,
        InferredGerberRole::TopMask,
    ));
    if pairs
        .iter()
        .all(|(silk, _)| roles[*silk] != InferredGerberRole::TopSilk)
    {
        pairs.extend(inferred_same_side_pairs(
            roles,
            InferredGerberRole::TopSilk,
            InferredGerberRole::TopCopper,
        ));
    }
    pairs.extend(inferred_same_side_pairs(
        roles,
        InferredGerberRole::BottomSilk,
        InferredGerberRole::BottomMask,
    ));
    if pairs
        .iter()
        .all(|(silk, _)| roles[*silk] != InferredGerberRole::BottomSilk)
    {
        pairs.extend(inferred_same_side_pairs(
            roles,
            InferredGerberRole::BottomSilk,
            InferredGerberRole::BottomCopper,
        ));
    }
    pairs
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
            let gerber_metadata_report = parse_gerber_metadata_report(&bytes);
            let source = file.source.clone().with_unit_context(
                gerber_metadata_report
                    .image_setup
                    .units
                    .map(gerber_units_label),
                Some("millimeters".to_string()),
            );
            Ok(Layer {
                path: file.path.clone(),
                source,
                gerber_image_setup: gerber_metadata_report.image_setup,
                gerber_metadata: gerber_metadata_report.metadata,
                gerber_aperture_definitions: gerber_metadata_report.aperture_definitions,
                gerber_aperture_macros: gerber_metadata_report.aperture_macros,
                gerber_aperture_uses: gerber_metadata_report.aperture_uses,
                gerber_coordinate_operations: gerber_metadata_report.coordinate_operations,
                gerber_polarity_changes: gerber_metadata_report.polarity_changes,
                gerber_image_transforms: gerber_metadata_report.image_transforms,
                gerber_region_events: gerber_metadata_report.region_events,
                gerber_step_repeat_events: gerber_metadata_report.step_repeat_events,
                gerber_interpolation_events: gerber_metadata_report.interpolation_events,
                gerber_quadrant_events: gerber_metadata_report.quadrant_events,
                gerber_object_metadata: gerber_metadata_report.object_attributes,
                gerber_attribute_deletes: gerber_metadata_report.attribute_deletes,
                gerber_metadata_issues: gerber_metadata_report.issues,
                sketch,
            })
        })
        .collect()
}

fn load_all_layers(
    files: &[PathBuf],
    gerber_dirs: &[PathBuf],
    extracted_packages: &[crate::package_archive::ExtractedPackage],
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
    for package in extracted_packages {
        layer_files.extend(io::discover_gerber_tree_from_archive(
            &package.directory,
            &package.archive,
        )?);
    }
    for output in conversion_outputs {
        for mut file in io::discover_gerber_dir(&output.gerber_dir)? {
            let mut transformation_history = output
                .steps
                .iter()
                .map(|step| step.command.clone())
                .collect::<Vec<_>>();
            if let Some(version) = &output.version {
                transformation_history.insert(0, version.command.clone());
            }
            file.source = io::converted_gerber_file(file.path.clone(), &output.source_dir)
                .source
                .with_transformation_context(
                    Some(output.input_hash.clone()),
                    transformation_history,
                );
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
                kicad_cli_bin: cli.kicad_cli_bin.clone(),
                extra_args: cli.conversion_args.clone(),
                kicad_drill_args: cli.kicad_cli_drill_args.clone(),
                kicad_pos_args: cli.kicad_cli_pos_args.clone(),
                kicad_ipcd356_args: cli.kicad_cli_ipcd356_args.clone(),
                kicad_drc_args: cli.kicad_cli_drc_args.clone(),
                kicad_handoff_exports: cli.kicad_cli_handoff_exports,
                kicad_review_exports: cli.kicad_cli_review_exports,
                kicad_dxf_args: cli.kicad_cli_dxf_args.clone(),
                kicad_svg_args: cli.kicad_cli_svg_args.clone(),
                kicad_pdf_args: cli.kicad_cli_pdf_args.clone(),
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
                part: layer.gerber_metadata.part.clone(),
                file_function: layer.gerber_metadata.file_function.clone(),
                file_polarity: layer.gerber_metadata.file_polarity.clone(),
                same_coordinates: layer.gerber_metadata.same_coordinates.clone(),
                creation_date: layer.gerber_metadata.creation_date.clone(),
                generation_software: layer.gerber_metadata.generation_software.clone(),
                project_id: layer.gerber_metadata.project_id.clone(),
                md5: layer.gerber_metadata.md5.clone(),
                units: layer.gerber_image_setup.units.map(gerber_units_label),
                coordinate_format: layer
                    .gerber_image_setup
                    .coordinate_format
                    .map(gerber_coordinate_format_label),
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

fn gerber_units_label(units: GerberUnits) -> String {
    match units {
        GerberUnits::Millimeters => "millimeters".to_string(),
        GerberUnits::Inches => "inches".to_string(),
    }
}

fn gerber_coordinate_format_label(format: GerberCoordinateFormat) -> String {
    format!("{}:{}", format.integer_digits, format.decimal_digits)
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
    layers: &[Layer],
    excellon_reports: &[excellon::ExcellonReport],
    ipc356_reports: &[ipc356::Ipc356Report],
    conversion_outputs: &[conversion::ConversionOutput],
    package_inputs: &PackageInputs,
) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();
    let gerber_object_attribute_count = layers
        .iter()
        .map(|layer| layer.gerber_object_metadata.len())
        .sum::<usize>();
    let gerber_attribute_delete_count = layers
        .iter()
        .map(|layer| layer.gerber_attribute_deletes.len())
        .sum::<usize>();
    let gerber_aperture_definition_count = layers
        .iter()
        .map(|layer| layer.gerber_aperture_definitions.len())
        .sum::<usize>();
    let gerber_aperture_macro_count = layers
        .iter()
        .map(|layer| layer.gerber_aperture_macros.len())
        .sum::<usize>();
    let gerber_aperture_use_count = layers
        .iter()
        .map(|layer| layer.gerber_aperture_uses.len())
        .sum::<usize>();
    let gerber_coordinate_operation_count = layers
        .iter()
        .map(|layer| layer.gerber_coordinate_operations.len())
        .sum::<usize>();
    let gerber_polarity_change_count = layers
        .iter()
        .map(|layer| layer.gerber_polarity_changes.len())
        .sum::<usize>();
    let gerber_image_transform_count = layers
        .iter()
        .map(|layer| layer.gerber_image_transforms.len())
        .sum::<usize>();
    let gerber_region_event_count = layers
        .iter()
        .map(|layer| layer.gerber_region_events.len())
        .sum::<usize>();
    let gerber_step_repeat_event_count = layers
        .iter()
        .map(|layer| layer.gerber_step_repeat_events.len())
        .sum::<usize>();
    let gerber_interpolation_event_count = layers
        .iter()
        .map(|layer| layer.gerber_interpolation_events.len())
        .sum::<usize>();
    let gerber_quadrant_event_count = layers
        .iter()
        .map(|layer| layer.gerber_quadrant_events.len())
        .sum::<usize>();
    let gerber_unit_count = layers
        .iter()
        .filter(|layer| layer.gerber_image_setup.units.is_some())
        .count();
    let gerber_format_count = layers
        .iter()
        .filter(|layer| layer.gerber_image_setup.coordinate_format.is_some())
        .count();
    log::trace!(
        "parser diagnostics input metadata: gerber_layers={} gerber_unit_declarations={} gerber_coordinate_formats={} gerber_aperture_definitions={} gerber_aperture_macros={} gerber_aperture_uses={} gerber_coordinate_operations={} gerber_polarity_changes={} gerber_image_transforms={} gerber_region_events={} gerber_step_repeat_events={} gerber_interpolation_events={} gerber_quadrant_events={} gerber_object_attributes={} gerber_attribute_deletes={}",
        layers.len(),
        gerber_unit_count,
        gerber_format_count,
        gerber_aperture_definition_count,
        gerber_aperture_macro_count,
        gerber_aperture_use_count,
        gerber_coordinate_operation_count,
        gerber_polarity_change_count,
        gerber_image_transform_count,
        gerber_region_event_count,
        gerber_step_repeat_event_count,
        gerber_interpolation_event_count,
        gerber_quadrant_event_count,
        gerber_object_attribute_count,
        gerber_attribute_delete_count
    );
    for layer in layers {
        diagnostics.extend(layer.gerber_metadata_issues.iter().map(|issue| Diagnostic {
            source: layer.source.path.clone(),
            line: Some(issue.line),
            severity: Severity::Warning,
            code: gerber_metadata_issue_code(&issue.kind).to_string(),
            message: issue.message(),
        }));
    }
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
    for output in conversion_outputs {
        diagnostics.extend(converter_output_diagnostics(output));
        if let Some(path) = &output.drc_report {
            diagnostics.extend(kicad_drc_report_diagnostics(path));
        }
    }
    diagnostics.extend(manufacturing_handoff_diagnostics(
        &package_inputs.manufacturing_handoff_files,
    ));
    diagnostics.extend(drawing_sidecar_parser_diagnostics(package_inputs));
    diagnostics.extend(text_package_parser_diagnostics(package_inputs));
    diagnostics
}

fn converter_output_diagnostics(output: &conversion::ConversionOutput) -> Vec<Diagnostic> {
    let source = output.gerber_dir.display().to_string();
    if !output.gerber_dir.exists() {
        return vec![Diagnostic {
            source,
            line: None,
            severity: Severity::Warning,
            code: "converter::output-dir-missing".to_string(),
            message: format!(
                "converter output directory for {} does not exist",
                output.source_dir.display()
            ),
        }];
    }
    match io::discover_gerber_dir(&output.gerber_dir) {
        Ok(files) if files.is_empty() => vec![Diagnostic {
            source,
            line: None,
            severity: Severity::Warning,
            code: "converter::output-dir-empty".to_string(),
            message: format!(
                "converter output directory for {} contains no Gerber layers",
                output.source_dir.display()
            ),
        }],
        Ok(_) => Vec::new(),
        Err(error) => vec![Diagnostic {
            source,
            line: None,
            severity: Severity::Warning,
            code: "converter::output-dir-unreadable".to_string(),
            message: format!("converter output directory could not be scanned: {error}"),
        }],
    }
}

fn manufacturing_handoff_diagnostics(files: &[io::DiscoveredFile]) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();
    for file in files {
        let source = file.path.display().to_string();
        let name = file
            .path
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or_default()
            .to_ascii_lowercase();
        let extension = file
            .path
            .extension()
            .and_then(|extension| extension.to_str())
            .unwrap_or_default()
            .to_ascii_lowercase();
        if matches!(extension.as_str(), "xml" | "ipc2581" | "cvg")
            || name.contains("ipc2581")
            || name.contains("ipc-2581")
            || name.contains("dpmx")
        {
            diagnostics.extend(ipc2581_handoff_diagnostics(&file.path, &source));
        } else if matches!(extension.as_str(), "gencad" | "gcd" | "cad")
            || name.contains("gencad")
            || name.contains("gen-cad")
        {
            diagnostics.extend(gencad_handoff_diagnostics(&file.path, &source));
        } else if is_test_inspection_handoff_name(&name) {
            diagnostics.extend(test_inspection_handoff_diagnostics(
                &file.path, &source, &name,
            ));
        } else if extension == "json"
            && (name.contains("statistics")
                || name.contains("stats")
                || name.contains("kicad-stats"))
        {
            diagnostics.extend(kicad_statistics_handoff_diagnostics(&file.path, &source));
        } else if is_review_image_handoff_extension(&extension) {
            diagnostics.extend(review_image_handoff_diagnostics(
                &file.path, &source, &extension,
            ));
        } else if is_mechanical_3d_handoff_extension(&extension) {
            diagnostics.extend(mechanical_3d_handoff_diagnostics(
                &file.path, &source, &extension,
            ));
        } else if name.contains("odb") {
            diagnostics.extend(odb_handoff_diagnostics(&file.path, &source, &extension));
        } else {
            diagnostics.push(Diagnostic {
                source,
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::unknown".to_string(),
                message: "manufacturing handoff sidecar is present, but no semantic importer is available for this filename/extension yet".to_string(),
            });
        }
    }
    diagnostics
}

fn odb_handoff_diagnostics(
    path: &std::path::Path,
    source: &str,
    extension: &str,
) -> Vec<Diagnostic> {
    let mut diagnostics = vec![Diagnostic {
        source: source.to_string(),
        line: None,
        severity: Severity::Warning,
        code: "manufacturing-handoff::odb-package".to_string(),
        message: "ODB++ manufacturing handoff package is present; HyperDRC records provenance and package tree evidence but does not import proprietary ODB++ stackup, net, or route semantics yet".to_string(),
    }];

    match odb_entry_names(path, extension) {
        Ok(entries) => {
            let summary = odb_tree_summary(&entries);
            diagnostics.push(Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::odb-summary".to_string(),
                message: format!(
                    "ODB++ package evidence: entries={} matrix={} steps={} layer-files={} feature-files={} profile-files={} netlist-like={} component-like={} drill-tool-like={}; review as provenance until a licensed ODB++ semantic importer is available",
                    summary.entry_count,
                    summary.matrix_present,
                    summary.step_count,
                    summary.layer_file_count,
                    summary.feature_file_count,
                    summary.profile_file_count,
                    summary.netlist_like_count,
                    summary.component_like_count,
                    summary.drill_tool_like_count
                ),
            });
            if let Ok(payloads) = odb_entry_payloads(path, extension) {
                let content = odb_content_summary(&payloads);
                if content.has_evidence() {
                    diagnostics.push(Diagnostic {
                        source: source.to_string(),
                        line: None,
                        severity: Severity::Warning,
                        code: "manufacturing-handoff::odb-content-summary".to_string(),
                        message: format!(
                            "ODB++ text payload evidence: matrix-records={} feature-records={} profile-records={} netlist-records={} component-records={} drill-tool-records={}; review as provenance until a licensed ODB++ semantic importer is available",
                            content.matrix_record_count,
                            content.feature_record_count,
                            content.profile_record_count,
                            content.netlist_record_count,
                            content.component_record_count,
                            content.drill_tool_record_count
                        ),
                    });
                }
            }
        }
        Err(error) => diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::odb-unreadable".to_string(),
            message: format!("ODB++ package tree could not be inspected: {error}"),
        }),
    }

    diagnostics
}

fn odb_entry_names(path: &std::path::Path, extension: &str) -> Result<Vec<String>> {
    use std::io::Read as _;

    if path.is_dir() {
        let mut entries = Vec::new();
        collect_directory_entry_names(path, path, &mut entries)?;
        return Ok(entries);
    }

    if extension == "zip" {
        let file = std::fs::File::open(path)
            .with_context(|| format!("failed to open {}", path.display()))?;
        let archive = zip::ZipArchive::new(file)
            .with_context(|| format!("failed to read ZIP archive {}", path.display()))?;
        return Ok(archive.file_names().map(ToOwned::to_owned).collect());
    }

    if extension == "tgz"
        || path
            .to_string_lossy()
            .to_ascii_lowercase()
            .ends_with(".tar.gz")
    {
        let file = std::fs::File::open(path)
            .with_context(|| format!("failed to open {}", path.display()))?;
        let decoder = flate2::read::GzDecoder::new(file);
        return tar_entry_names(decoder);
    }

    if extension == "tar" {
        let file = std::fs::File::open(path)
            .with_context(|| format!("failed to open {}", path.display()))?;
        return tar_entry_names(file);
    }

    let mut text = String::new();
    std::fs::File::open(path)
        .with_context(|| format!("failed to open {}", path.display()))?
        .read_to_string(&mut text)
        .with_context(|| format!("failed to read {}", path.display()))?;
    Ok(text.lines().map(str::to_string).collect())
}

fn odb_entry_payloads(path: &std::path::Path, extension: &str) -> Result<Vec<(String, String)>> {
    use std::io::Read as _;

    if path.is_dir() {
        let mut payloads = Vec::new();
        collect_directory_odb_payloads(path, path, &mut payloads)?;
        return Ok(payloads);
    }

    if extension == "zip" {
        let file = std::fs::File::open(path)
            .with_context(|| format!("failed to open {}", path.display()))?;
        let mut archive = zip::ZipArchive::new(file)
            .with_context(|| format!("failed to read ZIP archive {}", path.display()))?;
        let mut payloads = Vec::new();
        for index in 0..archive.len() {
            let mut file = archive
                .by_index(index)
                .context("failed to read ZIP archive entry")?;
            let name = file.name().replace('\\', "/");
            if !is_odb_text_payload_path(&name) {
                continue;
            }
            let mut text = String::new();
            std::io::Read::by_ref(&mut file)
                .take(64 * 1024)
                .read_to_string(&mut text)
                .with_context(|| format!("failed to read ODB++ payload {name}"))?;
            payloads.push((name, text));
        }
        return Ok(payloads);
    }

    if extension == "tgz"
        || path
            .to_string_lossy()
            .to_ascii_lowercase()
            .ends_with(".tar.gz")
    {
        let file = std::fs::File::open(path)
            .with_context(|| format!("failed to open {}", path.display()))?;
        let decoder = flate2::read::GzDecoder::new(file);
        return tar_odb_payloads(decoder);
    }

    if extension == "tar" {
        let file = std::fs::File::open(path)
            .with_context(|| format!("failed to open {}", path.display()))?;
        return tar_odb_payloads(file);
    }

    Ok(Vec::new())
}

fn collect_directory_entry_names(
    root: &std::path::Path,
    directory: &std::path::Path,
    entries: &mut Vec<String>,
) -> Result<()> {
    for entry in std::fs::read_dir(directory)
        .with_context(|| format!("failed to read {}", directory.display()))?
    {
        let entry =
            entry.with_context(|| format!("failed to read entry in {}", directory.display()))?;
        let path = entry.path();
        if path.is_dir() {
            collect_directory_entry_names(root, &path, entries)?;
        } else if path.is_file() {
            entries.push(
                path.strip_prefix(root)
                    .unwrap_or(&path)
                    .to_string_lossy()
                    .replace('\\', "/"),
            );
        }
    }
    Ok(())
}

fn collect_directory_odb_payloads(
    root: &std::path::Path,
    directory: &std::path::Path,
    payloads: &mut Vec<(String, String)>,
) -> Result<()> {
    use std::io::Read as _;

    for entry in std::fs::read_dir(directory)
        .with_context(|| format!("failed to read {}", directory.display()))?
    {
        let entry =
            entry.with_context(|| format!("failed to read entry in {}", directory.display()))?;
        let path = entry.path();
        if path.is_dir() {
            collect_directory_odb_payloads(root, &path, payloads)?;
        } else if path.is_file() {
            let name = path
                .strip_prefix(root)
                .unwrap_or(&path)
                .to_string_lossy()
                .replace('\\', "/");
            if is_odb_text_payload_path(&name) {
                let file = std::fs::File::open(&path)
                    .with_context(|| format!("failed to open {}", path.display()))?;
                let mut text = String::new();
                std::io::Read::take(file, 64 * 1024)
                    .read_to_string(&mut text)
                    .with_context(|| format!("failed to read {}", path.display()))?;
                payloads.push((name, text));
            }
        }
    }
    Ok(())
}

fn tar_entry_names<R: std::io::Read>(reader: R) -> Result<Vec<String>> {
    let mut archive = tar::Archive::new(reader);
    let mut entries = Vec::new();
    for entry in archive.entries().context("failed to read TAR entries")? {
        let entry = entry.context("failed to read TAR entry")?;
        entries.push(entry.path()?.to_string_lossy().replace('\\', "/"));
    }
    Ok(entries)
}

fn tar_odb_payloads<R: std::io::Read>(reader: R) -> Result<Vec<(String, String)>> {
    use std::io::Read as _;

    let mut archive = tar::Archive::new(reader);
    let mut payloads = Vec::new();
    for entry in archive.entries().context("failed to read TAR entries")? {
        let mut entry = entry.context("failed to read TAR entry")?;
        let name = entry.path()?.to_string_lossy().replace('\\', "/");
        if !is_odb_text_payload_path(&name) {
            continue;
        }
        let mut text = String::new();
        std::io::Read::by_ref(&mut entry)
            .take(64 * 1024)
            .read_to_string(&mut text)
            .with_context(|| format!("failed to read ODB++ payload {name}"))?;
        payloads.push((name, text));
    }
    Ok(payloads)
}

fn is_odb_text_payload_path(path: &str) -> bool {
    let normalized = path.trim_matches('/').to_ascii_lowercase();
    let parts = normalized.split('/').collect::<Vec<_>>();
    normalized == "matrix/matrix"
        || normalized.ends_with("/matrix/matrix")
        || parts.last() == Some(&"features")
        || parts.last() == Some(&"profile")
        || parts.last() == Some(&"netlist")
        || parts.last() == Some(&"data")
        || parts.last() == Some(&"tools")
}

#[derive(Default)]
struct OdbTreeSummary {
    entry_count: usize,
    matrix_present: bool,
    step_count: usize,
    layer_file_count: usize,
    feature_file_count: usize,
    profile_file_count: usize,
    netlist_like_count: usize,
    component_like_count: usize,
    drill_tool_like_count: usize,
}

fn odb_tree_summary(entries: &[String]) -> OdbTreeSummary {
    use std::collections::BTreeSet;

    let mut summary = OdbTreeSummary {
        entry_count: entries.len(),
        ..OdbTreeSummary::default()
    };
    let mut steps = BTreeSet::new();
    for entry in entries {
        let normalized = entry.trim_matches('/').to_ascii_lowercase();
        let parts = normalized.split('/').collect::<Vec<_>>();
        if normalized == "matrix/matrix" || normalized.ends_with("/matrix/matrix") {
            summary.matrix_present = true;
        }
        if let Some(index) = parts.iter().position(|part| *part == "steps") {
            if let Some(step) = parts.get(index + 1).filter(|step| !step.is_empty()) {
                steps.insert((*step).to_string());
            }
        }
        if normalized.contains("/layers/") || normalized.starts_with("layers/") {
            summary.layer_file_count += 1;
        }
        if parts.last() == Some(&"features") {
            summary.feature_file_count += 1;
        }
        if parts.last() == Some(&"profile") {
            summary.profile_file_count += 1;
        }
        if normalized.contains("netlist")
            || normalized.contains("/nets")
            || normalized.contains("/net/")
        {
            summary.netlist_like_count += 1;
        }
        if normalized.contains("component")
            || normalized.contains("/eda/")
            || normalized.contains("/bom")
        {
            summary.component_like_count += 1;
        }
        if normalized.contains("drill")
            || normalized.contains("tools")
            || normalized.contains("holes")
        {
            summary.drill_tool_like_count += 1;
        }
    }
    summary.step_count = steps.len();
    summary
}

#[derive(Default)]
struct OdbContentSummary {
    matrix_record_count: usize,
    feature_record_count: usize,
    profile_record_count: usize,
    netlist_record_count: usize,
    component_record_count: usize,
    drill_tool_record_count: usize,
}

impl OdbContentSummary {
    fn has_evidence(&self) -> bool {
        self.matrix_record_count
            + self.feature_record_count
            + self.profile_record_count
            + self.netlist_record_count
            + self.component_record_count
            + self.drill_tool_record_count
            > 0
    }
}

fn odb_content_summary(payloads: &[(String, String)]) -> OdbContentSummary {
    let mut summary = OdbContentSummary::default();
    for (path, text) in payloads {
        let normalized = path.trim_matches('/').to_ascii_lowercase();
        let parts = normalized.split('/').collect::<Vec<_>>();
        let record_count = text
            .lines()
            .map(str::trim)
            .filter(|line| !line.is_empty() && !line.starts_with('#') && !line.starts_with(';'))
            .count();
        if normalized == "matrix/matrix" || normalized.ends_with("/matrix/matrix") {
            summary.matrix_record_count += record_count;
        } else if parts.last() == Some(&"features") {
            summary.feature_record_count += record_count;
        } else if parts.last() == Some(&"profile") {
            summary.profile_record_count += record_count;
        } else if parts.last() == Some(&"netlist") {
            summary.netlist_record_count += record_count;
        } else if parts.last() == Some(&"data") {
            summary.component_record_count += record_count;
        } else if parts.last() == Some(&"tools") {
            summary.drill_tool_record_count += record_count;
        }
    }
    summary
}

fn is_review_image_handoff_extension(extension: &str) -> bool {
    matches!(
        extension,
        "png" | "jpg" | "jpeg" | "tif" | "tiff" | "bmp" | "webp"
    )
}

fn review_image_handoff_diagnostics(
    path: &std::path::Path,
    source: &str,
    extension: &str,
) -> Vec<Diagnostic> {
    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(error) => {
            return vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::review-image-unreadable".to_string(),
                message: format!("visual review image handoff could not be read: {error}"),
            }];
        }
    };
    let dimensions = image_header_dimensions(&bytes);
    let mut diagnostics = vec![Diagnostic {
        source: source.to_string(),
        line: None,
        severity: Severity::Warning,
        code: "manufacturing-handoff::review-image-present".to_string(),
        message: format!(
            "{} visual review image is present ({} bytes); HyperDRC records provenance but does not use raster images as dimensional source data",
            review_image_label(extension),
            bytes.len()
        ),
    }];
    match dimensions {
        Some((width, height)) => diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::review-image-summary".to_string(),
            message: format!(
                "visual review image parser evidence: {}; use for visual regression or review overlays only, not clearance or fabrication geometry",
                image_dimension_evidence(width, height)
            ),
        }),
        None => diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::review-image-unknown-dimensions".to_string(),
            message: "visual review image dimensions could not be read from the file header"
                .to_string(),
        }),
    }
    diagnostics
}

fn review_image_label(extension: &str) -> &'static str {
    match extension {
        "png" => "PNG",
        "jpg" | "jpeg" => "JPEG",
        "tif" | "tiff" => "TIFF",
        "bmp" => "BMP",
        "webp" => "WebP",
        _ => "raster",
    }
}

fn image_header_dimensions(bytes: &[u8]) -> Option<(u32, u32)> {
    png_dimensions(bytes)
        .or_else(|| jpeg_dimensions(bytes))
        .or_else(|| tiff_dimensions(bytes))
        .or_else(|| bmp_dimensions(bytes))
        .or_else(|| webp_dimensions(bytes))
}

fn image_dimension_evidence(width: u32, height: u32) -> String {
    let pixels = u64::from(width) * u64::from(height);
    let aspect_ratio = if height == 0 {
        0.0
    } else {
        f64::from(width) / f64::from(height)
    };
    format!("width={width}, height={height}, pixels={pixels}, aspect={aspect_ratio:.6}")
}

fn png_dimensions(bytes: &[u8]) -> Option<(u32, u32)> {
    if bytes.len() < 24 || &bytes[0..8] != b"\x89PNG\r\n\x1a\n" || &bytes[12..16] != b"IHDR" {
        return None;
    }
    Some((
        u32::from_be_bytes(bytes[16..20].try_into().ok()?),
        u32::from_be_bytes(bytes[20..24].try_into().ok()?),
    ))
}

fn jpeg_dimensions(bytes: &[u8]) -> Option<(u32, u32)> {
    if bytes.len() < 4 || bytes[0] != 0xff || bytes[1] != 0xd8 {
        return None;
    }
    let mut offset = 2usize;
    while offset + 9 <= bytes.len() {
        if bytes[offset] != 0xff {
            return None;
        }
        while offset < bytes.len() && bytes[offset] == 0xff {
            offset += 1;
        }
        if offset >= bytes.len() {
            return None;
        }
        let marker = bytes[offset];
        offset += 1;
        if matches!(marker, 0xd8 | 0xd9 | 0x01) {
            continue;
        }
        if offset + 2 > bytes.len() {
            return None;
        }
        let segment_len = u16::from_be_bytes(bytes[offset..offset + 2].try_into().ok()?) as usize;
        if segment_len < 2 || offset + segment_len > bytes.len() {
            return None;
        }
        if matches!(
            marker,
            0xc0 | 0xc1
                | 0xc2
                | 0xc3
                | 0xc5
                | 0xc6
                | 0xc7
                | 0xc9
                | 0xca
                | 0xcb
                | 0xcd
                | 0xce
                | 0xcf
        ) && segment_len >= 7
        {
            let height = u16::from_be_bytes(bytes[offset + 3..offset + 5].try_into().ok()?);
            let width = u16::from_be_bytes(bytes[offset + 5..offset + 7].try_into().ok()?);
            return Some((u32::from(width), u32::from(height)));
        }
        offset += segment_len;
    }
    None
}

fn tiff_dimensions(bytes: &[u8]) -> Option<(u32, u32)> {
    if bytes.len() < 8 {
        return None;
    }
    let little = if &bytes[0..2] == b"II" {
        true
    } else if &bytes[0..2] == b"MM" {
        false
    } else {
        return None;
    };
    if read_u16_endian(bytes, 2, little)? != 42 {
        return None;
    }
    let ifd_offset = read_u32_endian(bytes, 4, little)? as usize;
    if ifd_offset + 2 > bytes.len() {
        return None;
    }
    let entries = usize::from(read_u16_endian(bytes, ifd_offset, little)?);
    let mut width = None;
    let mut height = None;
    for index in 0..entries {
        let offset = ifd_offset + 2 + index * 12;
        if offset + 12 > bytes.len() {
            return None;
        }
        let tag = read_u16_endian(bytes, offset, little)?;
        let field_type = read_u16_endian(bytes, offset + 2, little)?;
        let value_count = read_u32_endian(bytes, offset + 4, little)?;
        if value_count != 1 || !matches!(tag, 256 | 257) {
            continue;
        }
        let value = match field_type {
            3 => u32::from(read_u16_endian(bytes, offset + 8, little)?),
            4 => read_u32_endian(bytes, offset + 8, little)?,
            _ => continue,
        };
        if tag == 256 {
            width = Some(value);
        } else {
            height = Some(value);
        }
    }
    Some((width?, height?))
}

fn bmp_dimensions(bytes: &[u8]) -> Option<(u32, u32)> {
    if bytes.len() < 26 || &bytes[0..2] != b"BM" {
        return None;
    }
    let width = i32::from_le_bytes(bytes[18..22].try_into().ok()?);
    let height = i32::from_le_bytes(bytes[22..26].try_into().ok()?);
    Some((width.unsigned_abs(), height.unsigned_abs()))
}

fn webp_dimensions(bytes: &[u8]) -> Option<(u32, u32)> {
    if bytes.len() < 30 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WEBP" {
        return None;
    }
    if &bytes[12..16] != b"VP8X" {
        return None;
    }
    let width =
        1 + u32::from(bytes[24]) + (u32::from(bytes[25]) << 8) + (u32::from(bytes[26]) << 16);
    let height =
        1 + u32::from(bytes[27]) + (u32::from(bytes[28]) << 8) + (u32::from(bytes[29]) << 16);
    Some((width, height))
}

fn read_u16_endian(bytes: &[u8], offset: usize, little: bool) -> Option<u16> {
    let value = bytes.get(offset..offset + 2)?.try_into().ok()?;
    Some(if little {
        u16::from_le_bytes(value)
    } else {
        u16::from_be_bytes(value)
    })
}

fn read_u32_endian(bytes: &[u8], offset: usize, little: bool) -> Option<u32> {
    let value = bytes.get(offset..offset + 4)?.try_into().ok()?;
    Some(if little {
        u32::from_le_bytes(value)
    } else {
        u32::from_be_bytes(value)
    })
}

fn kicad_statistics_handoff_diagnostics(path: &std::path::Path, source: &str) -> Vec<Diagnostic> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(error) => {
            return vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::kicad-statistics-unreadable".to_string(),
                message: format!("KiCad statistics handoff file could not be read: {error}"),
            }];
        }
    };
    let value: serde_json::Value = match serde_json::from_str(&text) {
        Ok(value) => value,
        Err(error) => {
            return vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::kicad-statistics-invalid".to_string(),
                message: format!("KiCad statistics handoff JSON could not be parsed: {error}"),
            }];
        }
    };
    let summary = json_value_summary(&value);
    let board_summary = kicad_statistics_domain_summary(&value);
    vec![
        Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::kicad-statistics-present".to_string(),
            message: "KiCad JSON statistics handoff is present; HyperDRC records parser evidence but does not import full board-statistics semantics yet".to_string(),
        },
        Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::kicad-statistics-summary".to_string(),
            message: format!(
                "KiCad statistics parser evidence: objects={}, arrays={}, numbers={}, strings={}, booleans={}, nulls={}, keys={}, layer-values={}, track-values={}, via-values={}, zone-arrays={}, filled-zone-values={}; review against board and manufacturing outputs until full statistics import is implemented",
                summary.object_count,
                summary.array_count,
                summary.number_count,
                summary.string_count,
                summary.boolean_count,
                summary.null_count,
                summary.key_count,
                board_summary.layer_value_count,
                board_summary.track_value_count,
                board_summary.via_value_count,
                board_summary.zone_array_count,
                board_summary.filled_zone_value_count
            ),
        },
    ]
}

#[derive(Default)]
struct JsonValueSummary {
    object_count: usize,
    array_count: usize,
    number_count: usize,
    string_count: usize,
    boolean_count: usize,
    null_count: usize,
    key_count: usize,
}

fn json_value_summary(value: &serde_json::Value) -> JsonValueSummary {
    fn visit(value: &serde_json::Value, summary: &mut JsonValueSummary) {
        match value {
            serde_json::Value::Object(object) => {
                summary.object_count += 1;
                summary.key_count += object.len();
                for value in object.values() {
                    visit(value, summary);
                }
            }
            serde_json::Value::Array(array) => {
                summary.array_count += 1;
                for value in array {
                    visit(value, summary);
                }
            }
            serde_json::Value::Number(_) => summary.number_count += 1,
            serde_json::Value::String(_) => summary.string_count += 1,
            serde_json::Value::Bool(_) => summary.boolean_count += 1,
            serde_json::Value::Null => summary.null_count += 1,
        }
    }

    let mut summary = JsonValueSummary::default();
    visit(value, &mut summary);
    summary
}

#[derive(Default)]
struct KiCadStatisticsDomainSummary {
    layer_value_count: usize,
    track_value_count: usize,
    via_value_count: usize,
    zone_array_count: usize,
    filled_zone_value_count: usize,
}

fn kicad_statistics_domain_summary(value: &serde_json::Value) -> KiCadStatisticsDomainSummary {
    fn visit(value: &serde_json::Value, summary: &mut KiCadStatisticsDomainSummary) {
        match value {
            serde_json::Value::Object(object) => {
                for (key, value) in object {
                    let normalized = key
                        .chars()
                        .filter(|character| character.is_ascii_alphanumeric())
                        .collect::<String>()
                        .to_ascii_lowercase();
                    if matches!(normalized.as_str(), "layers" | "layercount")
                        && value.as_u64().is_some()
                    {
                        summary.layer_value_count += 1;
                    }
                    if matches!(normalized.as_str(), "tracks" | "trackcount")
                        && value.as_u64().is_some()
                    {
                        summary.track_value_count += 1;
                    }
                    if matches!(normalized.as_str(), "vias" | "viacount")
                        && value.as_u64().is_some()
                    {
                        summary.via_value_count += 1;
                    }
                    if normalized == "zones" && value.as_array().is_some() {
                        summary.zone_array_count += 1;
                    }
                    if normalized == "filled" && value.as_bool().is_some() {
                        summary.filled_zone_value_count += 1;
                    }
                    visit(value, summary);
                }
            }
            serde_json::Value::Array(array) => {
                for value in array {
                    visit(value, summary);
                }
            }
            _ => {}
        }
    }

    let mut summary = KiCadStatisticsDomainSummary::default();
    visit(value, &mut summary);
    summary
}

fn is_mechanical_3d_handoff_extension(extension: &str) -> bool {
    matches!(
        extension,
        "step" | "stp" | "stepz" | "stl" | "obj" | "ply" | "glb" | "gltf" | "u3d"
    )
}

fn mechanical_3d_handoff_diagnostics(
    path: &std::path::Path,
    source: &str,
    extension: &str,
) -> Vec<Diagnostic> {
    let byte_len = std::fs::metadata(path).map(|metadata| metadata.len()).ok();
    let mut diagnostics = vec![Diagnostic {
        source: source.to_string(),
        line: None,
        severity: Severity::Warning,
        code: "manufacturing-handoff::mechanical-3d-present".to_string(),
        message: format!(
            "{} mechanical handoff is present{}; HyperDRC records provenance but does not import enclosure, B-rep, mesh, collision, or GD&T semantics yet",
            mechanical_3d_label(extension),
            byte_len
                .map(|bytes| format!(" ({bytes} bytes)"))
                .unwrap_or_default()
        ),
    }];
    match extension {
        "step" | "stp" => diagnostics.extend(step_handoff_summary_diagnostics(path, source)),
        "stl" => diagnostics.extend(stl_handoff_summary_diagnostics(path, source)),
        "obj" => diagnostics.extend(obj_handoff_summary_diagnostics(path, source)),
        "ply" => diagnostics.extend(ply_handoff_summary_diagnostics(path, source)),
        "glb" => diagnostics.extend(glb_handoff_summary_diagnostics(path, source)),
        "gltf" => diagnostics.extend(gltf_handoff_summary_diagnostics(path, source)),
        _ => {}
    }
    diagnostics
}

fn step_handoff_summary_diagnostics(path: &std::path::Path, source: &str) -> Vec<Diagnostic> {
    match std::fs::read_to_string(path) {
        Ok(text) => {
            let summary = step_text_summary(&text);
            vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::step-summary".to_string(),
                message: format!(
                    "STEP parser evidence: products={}, assemblies={}, solids={}, faces={}, shells={}, curves={}, placements={}, units={}, shape-representations={}, dimensions={}, tolerances={}; review against mechanical constraints until full STEP import is implemented",
                    summary.product_count,
                    summary.assembly_count,
                    summary.solid_count,
                    summary.face_count,
                    summary.shell_count,
                    summary.curve_count,
                    summary.placement_count,
                    summary.unit_count,
                    summary.shape_representation_count,
                    summary.dimension_count,
                    summary.tolerance_count
                ),
            }]
        }
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::step-unreadable".to_string(),
            message: format!("STEP mechanical handoff file could not be read: {error}"),
        }],
    }
}

fn stl_handoff_summary_diagnostics(path: &std::path::Path, source: &str) -> Vec<Diagnostic> {
    match std::fs::read(path) {
        Ok(bytes) => {
            let summary = stl_mesh_summary(&bytes);
            vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::stl-summary".to_string(),
                message: format!(
                    "STL mesh parser evidence: ascii={} triangles={} facets={} vertices={} bounds={}",
                    summary.is_ascii,
                    summary.triangle_count.unwrap_or(0),
                    summary.facet_count,
                    summary.vertex_count,
                    summary
                        .bounds
                        .as_ref()
                        .map(MeshBounds::to_evidence_string)
                        .unwrap_or_else(|| "<missing>".to_string())
                ),
            }]
        }
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::stl-unreadable".to_string(),
            message: format!("STL mechanical handoff file could not be read: {error}"),
        }],
    }
}

fn obj_handoff_summary_diagnostics(path: &std::path::Path, source: &str) -> Vec<Diagnostic> {
    match std::fs::read_to_string(path) {
        Ok(text) => {
            let summary = obj_mesh_summary(&text);
            vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::obj-summary".to_string(),
                message: format!(
                    "OBJ mesh parser evidence: objects={} groups={} vertices={} texture-vertices={} normals={} faces={} materials={} bounds={}",
                    summary.object_count,
                    summary.group_count,
                    summary.vertex_count,
                    summary.texture_vertex_count,
                    summary.normal_count,
                    summary.face_count,
                    summary.material_count,
                    summary
                        .bounds
                        .as_ref()
                        .map(MeshBounds::to_evidence_string)
                        .unwrap_or_else(|| "<missing>".to_string())
                ),
            }]
        }
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::obj-unreadable".to_string(),
            message: format!("OBJ mechanical handoff file could not be read: {error}"),
        }],
    }
}

fn ply_handoff_summary_diagnostics(path: &std::path::Path, source: &str) -> Vec<Diagnostic> {
    match std::fs::read_to_string(path) {
        Ok(text) => {
            let summary = ply_mesh_summary(&text);
            vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::ply-summary".to_string(),
                message: format!(
                    "PLY mesh parser evidence: format={:?} vertices={} faces={} properties={} bounds={}",
                    summary.format.unwrap_or_else(|| "unknown".to_string()),
                    summary.vertex_count.unwrap_or(0),
                    summary.face_count.unwrap_or(0),
                    summary.property_count,
                    summary
                        .bounds
                        .as_ref()
                        .map(MeshBounds::to_evidence_string)
                        .unwrap_or_else(|| "<missing>".to_string())
                ),
            }]
        }
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::ply-unreadable".to_string(),
            message: format!("PLY mechanical handoff file could not be read: {error}"),
        }],
    }
}

fn glb_handoff_summary_diagnostics(path: &std::path::Path, source: &str) -> Vec<Diagnostic> {
    match std::fs::read(path) {
        Ok(bytes) => match glb_mesh_summary(&bytes) {
            Some(summary) => vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::glb-summary".to_string(),
                message: format!(
                    "GLB mesh parser evidence: version={} declared-length={} bytes={}",
                    summary.version,
                    summary.declared_length,
                    bytes.len()
                ),
            }],
            None => vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::glb-invalid-header".to_string(),
                message: "GLB mechanical handoff header could not be parsed".to_string(),
            }],
        },
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::glb-unreadable".to_string(),
            message: format!("GLB mechanical handoff file could not be read: {error}"),
        }],
    }
}

fn gltf_handoff_summary_diagnostics(path: &std::path::Path, source: &str) -> Vec<Diagnostic> {
    match std::fs::read_to_string(path) {
        Ok(text) => match serde_json::from_str::<serde_json::Value>(&text) {
            Ok(value) => {
                let summary = gltf_json_summary(&value);
                vec![Diagnostic {
                    source: source.to_string(),
                    line: None,
                    severity: Severity::Warning,
                    code: "manufacturing-handoff::gltf-summary".to_string(),
                    message: format!(
                        "glTF parser evidence: scenes={} nodes={} meshes={} materials={} buffers={} buffer-views={} accessors={} images={} primitives={}",
                        summary.scene_count,
                        summary.node_count,
                        summary.mesh_count,
                        summary.material_count,
                        summary.buffer_count,
                        summary.buffer_view_count,
                        summary.accessor_count,
                        summary.image_count,
                        summary.primitive_count
                    ),
                }]
            }
            Err(error) => vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::gltf-invalid-json".to_string(),
                message: format!("glTF mechanical handoff JSON could not be parsed: {error}"),
            }],
        },
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::gltf-unreadable".to_string(),
            message: format!("glTF mechanical handoff file could not be read: {error}"),
        }],
    }
}

fn mechanical_3d_label(extension: &str) -> &'static str {
    match extension {
        "step" | "stp" | "stepz" => "STEP/STEPZ",
        "stl" => "STL mesh",
        "obj" => "OBJ mesh",
        "ply" => "PLY mesh",
        "glb" | "gltf" => "glTF/GLB mesh",
        "u3d" => "U3D mesh",
        _ => "3D mechanical",
    }
}

#[derive(Default)]
struct StepTextSummary {
    product_count: usize,
    assembly_count: usize,
    solid_count: usize,
    face_count: usize,
    shell_count: usize,
    curve_count: usize,
    placement_count: usize,
    unit_count: usize,
    shape_representation_count: usize,
    dimension_count: usize,
    tolerance_count: usize,
}

fn step_text_summary(text: &str) -> StepTextSummary {
    let mut summary = StepTextSummary::default();
    for line in text.lines() {
        let upper = line.to_ascii_uppercase();
        if upper.contains("PRODUCT(") || upper.contains("PRODUCT_DEFINITION(") {
            summary.product_count += 1;
        }
        if upper.contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE")
            || upper.contains("ASSEMBLY_COMPONENT_USAGE")
        {
            summary.assembly_count += 1;
        }
        if upper.contains("MANIFOLD_SOLID_BREP")
            || upper.contains("ADVANCED_BREP_SHAPE_REPRESENTATION")
            || upper.contains("BREP_WITH_VOIDS")
        {
            summary.solid_count += 1;
        }
        if upper.contains("ADVANCED_FACE") || upper.contains("FACE_BOUND") {
            summary.face_count += 1;
        }
        if upper.contains("CLOSED_SHELL") || upper.contains("OPEN_SHELL") {
            summary.shell_count += 1;
        }
        if upper.contains("B_SPLINE_CURVE")
            || upper.contains("CIRCLE(")
            || upper.contains("ELLIPSE(")
            || upper.contains("LINE(")
            || upper.contains("TRIMMED_CURVE")
        {
            summary.curve_count += 1;
        }
        if upper.contains("AXIS2_PLACEMENT") || upper.contains("CARTESIAN_POINT") {
            summary.placement_count += 1;
        }
        if upper.contains("SI_UNIT") || upper.contains("CONVERSION_BASED_UNIT") {
            summary.unit_count += 1;
        }
        if upper.contains("SHAPE_REPRESENTATION")
            || upper.contains("GEOMETRICALLY_BOUNDED")
            || upper.contains("MANIFOLD_SURFACE_SHAPE_REPRESENTATION")
        {
            summary.shape_representation_count += 1;
        }
        if upper.contains("DIMENSIONAL_SIZE")
            || upper.contains("DIMENSIONAL_LOCATION")
            || upper.contains("MEASURE_REPRESENTATION_ITEM")
        {
            summary.dimension_count += 1;
        }
        if upper.contains("GEOMETRIC_TOLERANCE")
            || upper.contains("TOLERANCE_VALUE")
            || upper.contains("PLUS_MINUS_TOLERANCE")
        {
            summary.tolerance_count += 1;
        }
    }
    summary
}

struct StlMeshSummary {
    is_ascii: bool,
    triangle_count: Option<usize>,
    facet_count: usize,
    vertex_count: usize,
    bounds: Option<MeshBounds>,
}

fn stl_mesh_summary(bytes: &[u8]) -> StlMeshSummary {
    let text = String::from_utf8_lossy(bytes);
    let is_ascii = text.trim_start().starts_with("solid") && text.contains("facet");
    if is_ascii {
        let mut bounds = MeshBounds::default();
        let mut vertex_count = 0;
        for line in text.lines().map(str::trim_start) {
            let Some(rest) = line.strip_prefix("vertex ") else {
                continue;
            };
            if let Some((x, y, z)) = parse_three_f64(rest) {
                vertex_count += 1;
                bounds.include(x, y, z);
            }
        }
        return StlMeshSummary {
            is_ascii: true,
            triangle_count: None,
            facet_count: text
                .lines()
                .filter(|line| line.trim_start().starts_with("facet normal"))
                .count(),
            vertex_count,
            bounds: bounds.finish(),
        };
    }
    let triangle_count = if bytes.len() >= 84 {
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap_or([0; 4]));
        Some(count as usize)
    } else {
        None
    };
    StlMeshSummary {
        is_ascii: false,
        triangle_count,
        facet_count: 0,
        vertex_count: triangle_count.map(|count| count * 3).unwrap_or(0),
        bounds: None,
    }
}

#[derive(Default)]
struct ObjMeshSummary {
    object_count: usize,
    group_count: usize,
    vertex_count: usize,
    texture_vertex_count: usize,
    normal_count: usize,
    face_count: usize,
    material_count: usize,
    bounds: Option<MeshBounds>,
}

fn obj_mesh_summary(text: &str) -> ObjMeshSummary {
    let mut summary = ObjMeshSummary::default();
    let mut bounds = MeshBounds::default();
    for line in text.lines().map(str::trim_start) {
        if line.starts_with("o ") {
            summary.object_count += 1;
        } else if line.starts_with("g ") {
            summary.group_count += 1;
        } else if line.starts_with("v ") {
            summary.vertex_count += 1;
            if let Some((x, y, z)) = parse_three_f64(line.trim_start_matches("v ")) {
                bounds.include(x, y, z);
            }
        } else if line.starts_with("vt ") {
            summary.texture_vertex_count += 1;
        } else if line.starts_with("vn ") {
            summary.normal_count += 1;
        } else if line.starts_with("f ") {
            summary.face_count += 1;
        } else if line.starts_with("usemtl ") || line.starts_with("mtllib ") {
            summary.material_count += 1;
        }
    }
    summary.bounds = bounds.finish();
    summary
}

#[derive(Default)]
struct PlyMeshSummary {
    format: Option<String>,
    vertex_count: Option<usize>,
    face_count: Option<usize>,
    property_count: usize,
    bounds: Option<MeshBounds>,
}

fn ply_mesh_summary(text: &str) -> PlyMeshSummary {
    let mut summary = PlyMeshSummary::default();
    let mut header_done = false;
    let mut vertex_rows_remaining = None;
    let mut vertex_property_names = Vec::new();
    let mut bounds = MeshBounds::default();
    for line in text.lines().map(str::trim) {
        if line == "end_header" {
            header_done = true;
            vertex_rows_remaining = summary.vertex_count;
            break;
        }
        let lower = line.to_ascii_lowercase();
        if let Some(rest) = lower.strip_prefix("format ") {
            summary.format = rest.split_ascii_whitespace().next().map(ToOwned::to_owned);
        } else if let Some(rest) = lower.strip_prefix("element vertex ") {
            summary.vertex_count = rest.trim().parse::<usize>().ok();
        } else if let Some(rest) = lower.strip_prefix("element face ") {
            summary.face_count = rest.trim().parse::<usize>().ok();
        } else if lower.starts_with("property ") {
            summary.property_count += 1;
            if summary.face_count.is_none() {
                vertex_property_names.push(
                    lower
                        .split_ascii_whitespace()
                        .last()
                        .unwrap_or_default()
                        .to_string(),
                );
            }
        }
    }
    if header_done && summary.format.as_deref() == Some("ascii") {
        let x_index = vertex_property_names.iter().position(|name| name == "x");
        let y_index = vertex_property_names.iter().position(|name| name == "y");
        let z_index = vertex_property_names.iter().position(|name| name == "z");
        if let (Some(x_index), Some(y_index), Some(z_index)) = (x_index, y_index, z_index) {
            let data_lines = text
                .lines()
                .map(str::trim)
                .skip_while(|line| *line != "end_header")
                .skip(1);
            for line in data_lines {
                let Some(remaining) = vertex_rows_remaining.as_mut() else {
                    break;
                };
                if *remaining == 0 {
                    break;
                }
                *remaining -= 1;
                let values = line.split_ascii_whitespace().collect::<Vec<_>>();
                let (Some(x), Some(y), Some(z)) = (
                    values.get(x_index),
                    values.get(y_index),
                    values.get(z_index),
                ) else {
                    continue;
                };
                if let (Ok(x), Ok(y), Ok(z)) =
                    (x.parse::<f64>(), y.parse::<f64>(), z.parse::<f64>())
                {
                    bounds.include(x, y, z);
                }
            }
        }
    }
    summary.bounds = bounds.finish();
    summary
}

#[derive(Clone, Debug, Default)]
struct MeshBounds {
    min_x: Option<f64>,
    min_y: Option<f64>,
    min_z: Option<f64>,
    max_x: Option<f64>,
    max_y: Option<f64>,
    max_z: Option<f64>,
}

impl MeshBounds {
    fn include(&mut self, x: f64, y: f64, z: f64) {
        if !(x.is_finite() && y.is_finite() && z.is_finite()) {
            return;
        }
        self.min_x = Some(self.min_x.map_or(x, |value| value.min(x)));
        self.min_y = Some(self.min_y.map_or(y, |value| value.min(y)));
        self.min_z = Some(self.min_z.map_or(z, |value| value.min(z)));
        self.max_x = Some(self.max_x.map_or(x, |value| value.max(x)));
        self.max_y = Some(self.max_y.map_or(y, |value| value.max(y)));
        self.max_z = Some(self.max_z.map_or(z, |value| value.max(z)));
    }

    fn finish(self) -> Option<Self> {
        if self.min_x.is_some() {
            Some(self)
        } else {
            None
        }
    }

    fn to_evidence_string(&self) -> String {
        format!(
            "{:.6},{:.6},{:.6}..{:.6},{:.6},{:.6}",
            self.min_x.unwrap_or(0.0),
            self.min_y.unwrap_or(0.0),
            self.min_z.unwrap_or(0.0),
            self.max_x.unwrap_or(0.0),
            self.max_y.unwrap_or(0.0),
            self.max_z.unwrap_or(0.0)
        )
    }
}

fn parse_three_f64(text: &str) -> Option<(f64, f64, f64)> {
    let mut values = text.split_ascii_whitespace();
    let x = values.next()?.parse::<f64>().ok()?;
    let y = values.next()?.parse::<f64>().ok()?;
    let z = values.next()?.parse::<f64>().ok()?;
    Some((x, y, z))
}

struct GlbMeshSummary {
    version: u32,
    declared_length: u32,
}

fn glb_mesh_summary(bytes: &[u8]) -> Option<GlbMeshSummary> {
    if bytes.len() < 12 || &bytes[0..4] != b"glTF" {
        return None;
    }
    Some(GlbMeshSummary {
        version: u32::from_le_bytes(bytes[4..8].try_into().ok()?),
        declared_length: u32::from_le_bytes(bytes[8..12].try_into().ok()?),
    })
}

#[derive(Default)]
struct GltfJsonSummary {
    scene_count: usize,
    node_count: usize,
    mesh_count: usize,
    material_count: usize,
    buffer_count: usize,
    buffer_view_count: usize,
    accessor_count: usize,
    image_count: usize,
    primitive_count: usize,
}

fn gltf_json_summary(value: &serde_json::Value) -> GltfJsonSummary {
    fn array_len(value: &serde_json::Value, key: &str) -> usize {
        value
            .get(key)
            .and_then(serde_json::Value::as_array)
            .map(Vec::len)
            .unwrap_or(0)
    }
    GltfJsonSummary {
        scene_count: array_len(value, "scenes"),
        node_count: array_len(value, "nodes"),
        mesh_count: array_len(value, "meshes"),
        material_count: array_len(value, "materials"),
        buffer_count: array_len(value, "buffers"),
        buffer_view_count: array_len(value, "bufferViews"),
        accessor_count: array_len(value, "accessors"),
        image_count: array_len(value, "images"),
        primitive_count: value
            .get("meshes")
            .and_then(serde_json::Value::as_array)
            .map(|meshes| {
                meshes
                    .iter()
                    .filter_map(|mesh| mesh.get("primitives"))
                    .filter_map(serde_json::Value::as_array)
                    .map(Vec::len)
                    .sum()
            })
            .unwrap_or(0),
    }
}

fn is_test_inspection_handoff_name(name: &str) -> bool {
    [
        "boundary-scan",
        "boundary_scan",
        "jtag",
        "flying-probe",
        "flying_probe",
        "aoi",
        "bed-of-nails",
        "bed_of_nails",
        "bon",
        "ict",
        "testpoint",
        "test-point",
    ]
    .iter()
    .any(|needle| name.contains(needle))
}

fn test_inspection_handoff_diagnostics(
    path: &std::path::Path,
    source: &str,
    name: &str,
) -> Vec<Diagnostic> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(error) => {
            return vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::test-inspection-unreadable".to_string(),
                message: format!("test/inspection handoff file could not be read: {error}"),
            }];
        }
    };
    let summary = test_inspection_text_summary(&text);
    let mut diagnostics = vec![
        Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::test-inspection-present".to_string(),
            message: format!(
                "{} handoff is present; HyperDRC records parser evidence but does not import boundary-scan, flying-probe, AOI, ICT, or fixture semantics yet",
                test_inspection_label(name)
            ),
        },
        Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::test-inspection-summary".to_string(),
            message: format!(
                "test/inspection parser evidence: lines={}, probe-like={}, net-like={}, component-like={}, pass/fail-like={}; review against design package outputs until a full tester-format importer is implemented",
                summary.line_count,
                summary.probe_like_count,
                summary.net_like_count,
                summary.component_like_count,
                summary.pass_fail_like_count
            ),
        },
    ];
    let extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    if name.contains("boundary") || name.contains("jtag") || extension == "svf" {
        let svf = boundary_scan_svf_summary(&text);
        if svf.command_count > 0 {
            diagnostics.push(Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::boundary-scan-summary".to_string(),
                message: format!(
                    "boundary-scan parser evidence: commands={} sir={} sdr={} runtest={} state={} trst={} tdi={} tdo={} masks={}",
                    svf.command_count,
                    svf.sir_count,
                    svf.sdr_count,
                    svf.runtest_count,
                    svf.state_count,
                    svf.trst_count,
                    svf.tdi_count,
                    svf.tdo_count,
                    svf.mask_count
                ),
            });
        }
    }
    if let Some(table) = test_inspection_table_summary(&text) {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::test-inspection-table-summary".to_string(),
            message: format!(
                "test/inspection table evidence: rows={} columns={} refdes-column={} net-column={} probe-column={} xy-columns={} result-column={} pass-results={} fail-results={} open-results={} short-results={}",
                table.row_count,
                table.column_count,
                table.has_refdes_column,
                table.has_net_column,
                table.has_probe_column,
                table.has_xy_columns,
                table.has_result_column,
                table.pass_result_count,
                table.fail_result_count,
                table.open_result_count,
                table.short_result_count
            ),
        });
    }
    diagnostics
}

fn test_inspection_label(name: &str) -> &'static str {
    if name.contains("boundary") || name.contains("jtag") {
        "boundary-scan"
    } else if name.contains("flying") {
        "flying-probe"
    } else if name.contains("aoi") {
        "AOI"
    } else if name.contains("bed") || name.contains("bon") || name.contains("ict") {
        "bed-of-nails/ICT"
    } else {
        "test/inspection"
    }
}

#[derive(Default)]
struct TestInspectionTextSummary {
    line_count: usize,
    probe_like_count: usize,
    net_like_count: usize,
    component_like_count: usize,
    pass_fail_like_count: usize,
}

fn test_inspection_text_summary(text: &str) -> TestInspectionTextSummary {
    let mut summary = TestInspectionTextSummary::default();
    for line in text.lines() {
        let lower = line.trim().to_ascii_lowercase();
        if lower.is_empty() {
            continue;
        }
        summary.line_count += 1;
        if lower.contains("probe") || lower.contains("testpoint") || lower.contains("test point") {
            summary.probe_like_count += 1;
        }
        if lower.contains("net") || lower.contains("signal") {
            summary.net_like_count += 1;
        }
        if lower.contains("refdes")
            || lower.contains("component")
            || lower.contains("part")
            || lower.contains("pin")
        {
            summary.component_like_count += 1;
        }
        if lower.contains("pass")
            || lower.contains("fail")
            || lower.contains("open")
            || lower.contains("short")
        {
            summary.pass_fail_like_count += 1;
        }
    }
    summary
}

#[derive(Default)]
struct BoundaryScanSvfSummary {
    command_count: usize,
    sir_count: usize,
    sdr_count: usize,
    runtest_count: usize,
    state_count: usize,
    trst_count: usize,
    tdi_count: usize,
    tdo_count: usize,
    mask_count: usize,
}

fn boundary_scan_svf_summary(text: &str) -> BoundaryScanSvfSummary {
    let mut summary = BoundaryScanSvfSummary::default();
    for command in text.split(';') {
        let upper = command.trim().to_ascii_uppercase();
        if upper.is_empty() || upper.starts_with('!') {
            continue;
        }
        let keyword = upper.split_ascii_whitespace().next().unwrap_or_default();
        match keyword {
            "SIR" => summary.sir_count += 1,
            "SDR" => summary.sdr_count += 1,
            "RUNTEST" => summary.runtest_count += 1,
            "STATE" => summary.state_count += 1,
            "TRST" => summary.trst_count += 1,
            _ => {}
        }
        if matches!(keyword, "SIR" | "SDR" | "RUNTEST" | "STATE" | "TRST") {
            summary.command_count += 1;
        }
        if upper.contains("TDI") {
            summary.tdi_count += 1;
        }
        if upper.contains("TDO") {
            summary.tdo_count += 1;
        }
        if upper.contains("MASK") || upper.contains("SMASK") {
            summary.mask_count += 1;
        }
    }
    summary
}

struct TestInspectionTableSummary {
    row_count: usize,
    column_count: usize,
    has_refdes_column: bool,
    has_net_column: bool,
    has_probe_column: bool,
    has_xy_columns: bool,
    has_result_column: bool,
    pass_result_count: usize,
    fail_result_count: usize,
    open_result_count: usize,
    short_result_count: usize,
}

fn test_inspection_table_summary(text: &str) -> Option<TestInspectionTableSummary> {
    let lines = text
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .collect::<Vec<_>>();
    let header = *lines.first()?;
    let delimiter = if header.contains('\t') {
        '\t'
    } else if header.contains(',') {
        ','
    } else if header.contains(';') {
        ';'
    } else {
        return None;
    };
    let headers = split_simple_table_row(header, delimiter)
        .into_iter()
        .map(|header| normalize_test_inspection_header(&header))
        .collect::<Vec<_>>();
    if headers.len() < 2 {
        return None;
    }
    let row_count = lines
        .iter()
        .skip(1)
        .filter(|line| {
            split_simple_table_row(line, delimiter)
                .iter()
                .any(|cell| !cell.trim().is_empty())
        })
        .count();
    let has_x = headers.iter().any(|header| {
        matches!(
            header.as_str(),
            "x" | "xmm" | "xmil" | "posx" | "locationx" | "probex"
        )
    });
    let has_y = headers.iter().any(|header| {
        matches!(
            header.as_str(),
            "y" | "ymm" | "ymil" | "posy" | "locationy" | "probey"
        )
    });
    let result_index = headers.iter().position(|header| {
        matches!(
            header.as_str(),
            "result" | "status" | "outcome" | "measurement" | "inspectionresult"
        )
    });
    let mut pass_result_count = 0;
    let mut fail_result_count = 0;
    let mut open_result_count = 0;
    let mut short_result_count = 0;
    if let Some(result_index) = result_index {
        for row in lines.iter().skip(1) {
            let cells = split_simple_table_row(row, delimiter);
            let Some(result) = cells.get(result_index) else {
                continue;
            };
            let result = result.trim_matches('"').to_ascii_lowercase();
            if result.contains("pass") || result == "ok" {
                pass_result_count += 1;
            }
            if result.contains("fail") || result.contains("error") {
                fail_result_count += 1;
            }
            if result.contains("open") {
                open_result_count += 1;
            }
            if result.contains("short") {
                short_result_count += 1;
            }
        }
    }
    Some(TestInspectionTableSummary {
        row_count,
        column_count: headers.len(),
        has_refdes_column: headers.iter().any(|header| {
            matches!(
                header.as_str(),
                "refdes" | "reference" | "designator" | "component" | "part"
            )
        }),
        has_net_column: headers
            .iter()
            .any(|header| matches!(header.as_str(), "net" | "signal" | "netname")),
        has_probe_column: headers.iter().any(|header| {
            matches!(
                header.as_str(),
                "probe" | "probeid" | "testpoint" | "testpointid" | "tp" | "fixturepin"
            )
        }),
        has_xy_columns: has_x && has_y,
        has_result_column: result_index.is_some(),
        pass_result_count,
        fail_result_count,
        open_result_count,
        short_result_count,
    })
}

fn split_simple_table_row(line: &str, delimiter: char) -> Vec<String> {
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

fn normalize_test_inspection_header(header: &str) -> String {
    header
        .trim_matches('"')
        .chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .collect::<String>()
        .to_ascii_lowercase()
}

fn gencad_handoff_diagnostics(path: &std::path::Path, source: &str) -> Vec<Diagnostic> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(error) => {
            return vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::gencad-unreadable".to_string(),
                message: format!("GenCAD handoff file could not be read: {error}"),
            }];
        }
    };
    let summary = gencad_text_summary(&text);
    let mut diagnostics = vec![Diagnostic {
        source: source.to_string(),
        line: None,
        severity: Severity::Warning,
        code: "manufacturing-handoff::gencad-present".to_string(),
        message: "GenCAD manufacturing/test handoff is present; HyperDRC records parser evidence but does not import full component, route, or fixture semantics yet".to_string(),
    }];
    if summary.section_count == 0 {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::gencad-no-sections".to_string(),
            message: "GenCAD handoff file contains no recognizable $SECTION headers".to_string(),
        });
    } else {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::gencad-summary".to_string(),
            message: format!(
                "GenCAD parser evidence: sections={}, board={}, components={}, parts={}, signals={}, routes={}, test={}, component-records={}, part-records={}, signal-records={}, route-records={}, testpoint-records={}, unique-components={}, unique-parts={}, unique-signals={}, unique-testpoints={}, coordinate-like-records={}; review against Gerber/drill/BOM sidecars until full GenCAD import is implemented",
                summary.section_count,
                summary.board_sections,
                summary.component_sections,
                summary.part_sections,
                summary.signal_sections,
                summary.route_sections,
                summary.test_sections,
                summary.component_records,
                summary.part_records,
                summary.signal_records,
                summary.route_records,
                summary.testpoint_records,
                summary.unique_component_count,
                summary.unique_part_count,
                summary.unique_signal_count,
                summary.unique_testpoint_count,
                summary.coordinate_like_record_count
            ),
        });
    }
    diagnostics
}

#[derive(Default)]
struct GencadTextSummary {
    section_count: usize,
    board_sections: usize,
    component_sections: usize,
    part_sections: usize,
    signal_sections: usize,
    route_sections: usize,
    test_sections: usize,
    component_records: usize,
    part_records: usize,
    signal_records: usize,
    route_records: usize,
    testpoint_records: usize,
    unique_component_count: usize,
    unique_part_count: usize,
    unique_signal_count: usize,
    unique_testpoint_count: usize,
    coordinate_like_record_count: usize,
}

fn gencad_text_summary(text: &str) -> GencadTextSummary {
    use std::collections::BTreeSet;

    let mut summary = GencadTextSummary::default();
    let mut current_section = String::new();
    let mut component_ids = BTreeSet::new();
    let mut part_ids = BTreeSet::new();
    let mut signal_ids = BTreeSet::new();
    let mut testpoint_ids = BTreeSet::new();
    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with(';') || trimmed.starts_with('#') {
            continue;
        }
        if let Some(rest) = trimmed.strip_prefix('$') {
            let section = rest
                .split(|character: char| character.is_whitespace() || character == ';')
                .next()
                .unwrap_or_default()
                .to_ascii_lowercase();
            if section.is_empty() {
                continue;
            }
            if section == "end" || section.starts_with("end") {
                current_section.clear();
                continue;
            }
            current_section = section.clone();
            summary.section_count += 1;
            if section.contains("board") {
                summary.board_sections += 1;
            }
            if section.contains("component") || section.contains("comp") {
                summary.component_sections += 1;
            }
            if section.contains("part") || section.contains("package") {
                summary.part_sections += 1;
            }
            if section.contains("signal") || section.contains("net") {
                summary.signal_sections += 1;
            }
            if section.contains("route") || section.contains("track") {
                summary.route_sections += 1;
            }
            if section.contains("test") || section.contains("probe") || section.contains("fixture")
            {
                summary.test_sections += 1;
            }
            continue;
        }
        if !gencad_data_record_like(trimmed) {
            continue;
        }
        let record_id = trimmed
            .split(|character: char| {
                character.is_whitespace() || character == ',' || character == ';'
            })
            .next()
            .unwrap_or_default()
            .trim_matches('"')
            .to_string();
        if gencad_coordinate_like_record(trimmed) {
            summary.coordinate_like_record_count += 1;
        }
        if current_section.contains("component") || current_section.contains("comp") {
            summary.component_records += 1;
            if !record_id.is_empty() {
                component_ids.insert(record_id);
            }
        } else if current_section.contains("part") || current_section.contains("package") {
            summary.part_records += 1;
            if !record_id.is_empty() {
                part_ids.insert(record_id);
            }
        } else if current_section.contains("signal") || current_section.contains("net") {
            summary.signal_records += 1;
            if !record_id.is_empty() {
                signal_ids.insert(record_id);
            }
        } else if current_section.contains("route") || current_section.contains("track") {
            summary.route_records += 1;
        } else if current_section.contains("test")
            || current_section.contains("probe")
            || current_section.contains("fixture")
        {
            summary.testpoint_records += 1;
            if !record_id.is_empty() {
                testpoint_ids.insert(record_id);
            }
        }
    }
    summary.unique_component_count = component_ids.len();
    summary.unique_part_count = part_ids.len();
    summary.unique_signal_count = signal_ids.len();
    summary.unique_testpoint_count = testpoint_ids.len();
    summary
}

fn gencad_data_record_like(line: &str) -> bool {
    let keyword = line
        .split(|character: char| character.is_whitespace() || character == ',' || character == ';')
        .next()
        .unwrap_or_default()
        .to_ascii_lowercase();
    !keyword.is_empty()
        && !matches!(
            keyword.as_str(),
            "comment" | "user" | "units" | "unit" | "version" | "generated"
        )
}

fn gencad_coordinate_like_record(line: &str) -> bool {
    line.split(|character: char| character.is_whitespace() || character == ',' || character == ';')
        .filter(|token| token.parse::<f64>().is_ok())
        .take(2)
        .count()
        >= 2
}

fn ipc2581_handoff_diagnostics(path: &std::path::Path, source: &str) -> Vec<Diagnostic> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(error) => {
            return vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::ipc2581-unreadable".to_string(),
                message: format!("IPC-2581 handoff file could not be read: {error}"),
            }];
        }
    };
    match ipc2581_xml_summary(&text) {
        Ok(summary) => {
            let mut diagnostics = vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "manufacturing-handoff::ipc2581-present".to_string(),
                message: format!(
                    "IPC-2581/DPMX manufacturing handoff is present with root {}; HyperDRC records parser evidence but does not import full manufacturing geometry yet",
                    summary.root_name.unwrap_or_else(|| "unknown".to_string())
                ),
            }];
            if summary.stackup_like_count > 0
                || summary.layer_like_count > 0
                || summary.net_like_count > 0
                || summary.component_like_count > 0
                || summary.package_like_count > 0
            {
                diagnostics.push(Diagnostic {
                    source: source.to_string(),
                    line: None,
                    severity: Severity::Warning,
                    code: "manufacturing-handoff::ipc2581-summary".to_string(),
                    message: format!(
                        "IPC-2581/DPMX parser evidence: layers={}, stackup-like={}, nets={}, components={}, packages={}; review against Gerber/drill/BOM sidecars until full IPC-2581 import is implemented",
                        summary.layer_like_count,
                        summary.stackup_like_count,
                        summary.net_like_count,
                        summary.component_like_count,
                        summary.package_like_count
                    ),
                });
            }
            if summary.named_layer_count > 0
                || summary.named_net_count > 0
                || summary.refdes_count > 0
                || summary.package_name_count > 0
                || summary.drill_like_count > 0
                || summary.coordinate_like_count > 0
                || summary.unit_like_count > 0
                || summary.material_like_count > 0
                || summary.thickness_like_count > 0
                || summary.material_name_count > 0
                || summary.thickness_value_count > 0
            {
                diagnostics.push(Diagnostic {
                    source: source.to_string(),
                    line: None,
                    severity: Severity::Warning,
                    code: "manufacturing-handoff::ipc2581-design-evidence".to_string(),
                    message: format!(
                        "IPC-2581/DPMX design evidence: named-layers={}, named-nets={}, refdes={}, package-names={}, drill-like={}, coordinate-like={}, unit-like={}, material-like={}, material-names={}, thickness-like={}, thickness-values={}; evidence is informational until full IPC-2581 object import is implemented",
                        summary.named_layer_count,
                        summary.named_net_count,
                        summary.refdes_count,
                        summary.package_name_count,
                        summary.drill_like_count,
                        summary.coordinate_like_count,
                        summary.unit_like_count,
                        summary.material_like_count,
                        summary.material_name_count,
                        summary.thickness_like_count,
                        summary.thickness_value_count
                    ),
                });
            }
            diagnostics
        }
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "manufacturing-handoff::ipc2581-xml-invalid".to_string(),
            message: format!("IPC-2581 handoff XML could not be parsed: {error}"),
        }],
    }
}

#[derive(Default)]
struct Ipc2581XmlSummary {
    root_name: Option<String>,
    layer_like_count: usize,
    stackup_like_count: usize,
    net_like_count: usize,
    component_like_count: usize,
    package_like_count: usize,
    named_layer_count: usize,
    named_net_count: usize,
    refdes_count: usize,
    package_name_count: usize,
    drill_like_count: usize,
    coordinate_like_count: usize,
    unit_like_count: usize,
    material_like_count: usize,
    thickness_like_count: usize,
    material_name_count: usize,
    thickness_value_count: usize,
}

fn ipc2581_xml_summary(text: &str) -> Result<Ipc2581XmlSummary, quick_xml::Error> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader as XmlReader;
    use std::collections::BTreeSet;

    let mut summary = Ipc2581XmlSummary::default();
    let mut layer_names = BTreeSet::new();
    let mut net_names = BTreeSet::new();
    let mut refdes_values = BTreeSet::new();
    let mut package_names = BTreeSet::new();
    let mut material_names = BTreeSet::new();
    let mut thickness_values = BTreeSet::new();
    let mut reader = XmlReader::from_str(text);
    reader.config_mut().trim_text(true);
    loop {
        match reader.read_event()? {
            Event::Start(element) | Event::Empty(element) => {
                let name = String::from_utf8_lossy(element.local_name().as_ref()).to_string();
                if summary.root_name.is_none() {
                    summary.root_name = Some(name.clone());
                }
                let lower = name.to_ascii_lowercase();
                if lower.contains("layer") {
                    summary.layer_like_count += 1;
                }
                if lower.contains("stackup") || lower.contains("stack") {
                    summary.stackup_like_count += 1;
                }
                if lower.contains("material")
                    || lower.contains("dielectric")
                    || lower.contains("laminate")
                    || lower.contains("prepreg")
                    || lower.contains("core")
                {
                    summary.material_like_count += 1;
                }
                if lower.contains("net") {
                    summary.net_like_count += 1;
                }
                if lower.contains("component") || lower.contains("part") {
                    summary.component_like_count += 1;
                }
                if lower.contains("package") || lower.contains("bom") {
                    summary.package_like_count += 1;
                }
                for attribute in element.attributes().with_checks(false).flatten() {
                    let key = String::from_utf8_lossy(attribute.key.local_name().as_ref())
                        .to_ascii_lowercase();
                    let Ok(value) = attribute.decode_and_unescape_value(reader.decoder()) else {
                        continue;
                    };
                    let value = value.trim();
                    if value.is_empty() {
                        continue;
                    }
                    if (lower.contains("layer") || key.contains("layer"))
                        && matches!(
                            key.as_str(),
                            "name" | "layer" | "layerref" | "layerrefdes" | "id"
                        )
                    {
                        layer_names.insert(value.to_string());
                    }
                    if (lower.contains("net") || key.contains("net"))
                        && matches!(key.as_str(), "name" | "net" | "netname" | "id")
                    {
                        net_names.insert(value.to_string());
                    }
                    if matches!(
                        key.as_str(),
                        "refdes" | "ref" | "componentref" | "component" | "designator"
                    ) {
                        refdes_values.insert(value.to_string());
                    }
                    if (lower.contains("package") || lower.contains("part") || key.contains("part"))
                        && matches!(
                            key.as_str(),
                            "name" | "package" | "packagename" | "part" | "partname" | "id"
                        )
                    {
                        package_names.insert(value.to_string());
                    }
                    if key.contains("diam")
                        || key.contains("hole")
                        || key.contains("drill")
                        || lower.contains("drill")
                        || lower.contains("hole")
                    {
                        summary.drill_like_count += 1;
                    }
                    if matches!(key.as_str(), "x" | "y" | "x1" | "y1" | "x2" | "y2")
                        || key.contains("coord")
                        || key.contains("location")
                        || key.contains("position")
                    {
                        summary.coordinate_like_count += 1;
                    }
                    if key.contains("unit")
                        || matches!(
                            value.to_ascii_lowercase().as_str(),
                            "mm" | "millimeter" | "millimeters" | "inch" | "in" | "mil"
                        )
                    {
                        summary.unit_like_count += 1;
                    }
                    if key.contains("material")
                        || key.contains("dielectric")
                        || key.contains("laminate")
                        || key.contains("prepreg")
                        || key.contains("core")
                    {
                        summary.material_like_count += 1;
                    }
                    if (lower.contains("material")
                        || lower.contains("dielectric")
                        || lower.contains("laminate")
                        || lower.contains("prepreg")
                        || lower.contains("core")
                        || key.contains("material"))
                        && matches!(
                            key.as_str(),
                            "name" | "material" | "materialname" | "materialref" | "id"
                        )
                    {
                        material_names.insert(value.to_string());
                    }
                    if key.contains("thick")
                        || key.contains("height")
                        || key.contains("copperweight")
                        || key.contains("weight")
                    {
                        summary.thickness_like_count += 1;
                        thickness_values.insert(value.to_string());
                    }
                }
            }
            Event::Eof => break,
            _ => {}
        }
    }
    summary.named_layer_count = layer_names.len();
    summary.named_net_count = net_names.len();
    summary.refdes_count = refdes_values.len();
    summary.package_name_count = package_names.len();
    summary.material_name_count = material_names.len();
    summary.thickness_value_count = thickness_values.len();
    Ok(summary)
}

fn text_package_parser_diagnostics(package_inputs: &PackageInputs) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();
    diagnostics.extend(text_artifact_kind_diagnostics(
        &package_inputs.bom_files,
        checks::TextArtifactParserKind::Bom,
    ));
    diagnostics.extend(text_artifact_kind_diagnostics(
        &package_inputs.centroid_files,
        checks::TextArtifactParserKind::Centroid,
    ));
    diagnostics.extend(text_artifact_kind_diagnostics(
        &package_inputs.netlist_files,
        checks::TextArtifactParserKind::Netlist,
    ));
    diagnostics
}

fn drawing_sidecar_parser_diagnostics(package_inputs: &PackageInputs) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();
    diagnostics.extend(drawing_artifact_kind_diagnostics(
        &package_inputs.fab_drawing_files,
        "fabrication drawing",
    ));
    diagnostics.extend(drawing_artifact_kind_diagnostics(
        &package_inputs.assembly_drawing_files,
        "assembly drawing",
    ));
    diagnostics.extend(drawing_artifact_kind_diagnostics(
        &package_inputs.rout_drawing_files,
        "rout/panel drawing",
    ));
    diagnostics
}

fn drawing_artifact_kind_diagnostics(
    files: &[io::DiscoveredFile],
    role_label: &str,
) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();
    for file in files {
        let source = file.path.display().to_string();
        let extension = file
            .path
            .extension()
            .and_then(|extension| extension.to_str())
            .unwrap_or_default()
            .to_ascii_lowercase();
        match extension.as_str() {
            "svg" => diagnostics.extend(svg_drawing_diagnostics(&file.path, role_label, &source)),
            "dxf" => diagnostics.extend(dxf_drawing_diagnostics(&file.path, role_label, &source)),
            "pdf" => diagnostics.extend(pdf_drawing_diagnostics(&file.path, role_label, &source)),
            "ps" | "eps" => {
                diagnostics.extend(postscript_drawing_diagnostics(&file.path, role_label, &source))
            }
            "plt" | "hpgl" | "hpg" => {
                diagnostics.extend(hpgl_drawing_diagnostics(&file.path, role_label, &source))
            }
            "png" | "jpg" | "jpeg" | "tif" | "tiff" | "bmp" | "webp" => {
                diagnostics.extend(raster_drawing_diagnostics(
                    &file.path,
                    role_label,
                    &source,
                    &extension,
                ))
            }
            "dwg" | "dxb" | "sat" | "sab" | "acis" => diagnostics.push(Diagnostic {
                source,
                line: None,
                severity: Severity::Warning,
                code: "drawing::binary-cad-present".to_string(),
                message: format!(
                    "{role_label} CAD sidecar uses {extension:?}; HyperDRC retains it in the input manifest but does not import binary/mechanical CAD geometry into checks yet"
                ),
            }),
            _ => {}
        }
    }
    diagnostics
}

fn pdf_drawing_diagnostics(
    path: &std::path::Path,
    role_label: &str,
    source: &str,
) -> Vec<Diagnostic> {
    match std::fs::read(path) {
        Ok(bytes) => {
            let text = String::from_utf8_lossy(&bytes);
            let page_marker_count = text.matches("/Page").count();
            let annotation_count = text.matches("/Annots").count();
            let image_count = text.matches("/Image").count();
            let stream_count = text.matches("stream").count();
            let font_count = text.matches("/Font").count();
            let xobject_count = text.matches("/XObject").count();
            let text_object_count = text.matches("BT").count();
            let text_show_count = text.matches(" Tj").count() + text.matches(" TJ").count();
            let media_box_count = text.matches("/MediaBox").count();
            let mut diagnostics = vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "drawing::pdf-summary".to_string(),
                message: format!(
                    "{role_label} PDF parser evidence: bytes={} page-markers={} annotations={} images={} streams={} fonts={} xobjects={} text-objects={} text-shows={} media-boxes={}",
                    bytes.len(),
                    page_marker_count,
                    annotation_count,
                    image_count,
                    stream_count,
                    font_count,
                    xobject_count,
                    text_object_count,
                    text_show_count,
                    media_box_count
                ),
            }];
            if !bytes.starts_with(b"%PDF-") {
                diagnostics.push(Diagnostic {
                    source: source.to_string(),
                    line: None,
                    severity: Severity::Warning,
                    code: "drawing::pdf-invalid-header".to_string(),
                    message: format!("{role_label} PDF does not start with a %PDF header"),
                });
            }
            diagnostics
        }
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "drawing::pdf-unreadable".to_string(),
            message: format!("{role_label} PDF sidecar could not be read: {error}"),
        }],
    }
}

fn postscript_drawing_diagnostics(
    path: &std::path::Path,
    role_label: &str,
    source: &str,
) -> Vec<Diagnostic> {
    match std::fs::read_to_string(path) {
        Ok(text) => {
            let lower = text.to_ascii_lowercase();
            let line_count = lower.lines().count();
            let bounding_box_count = lower.matches("%%boundingbox:").count();
            let moveto_count = lower.matches(" moveto").count() + lower.matches("\nmoveto").count();
            let lineto_count = lower.matches(" lineto").count() + lower.matches("\nlineto").count();
            let stroke_count = lower.matches(" stroke").count() + lower.matches("\nstroke").count();
            let showpage_count = lower.matches("showpage").count();
            vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "drawing::postscript-summary".to_string(),
                message: format!(
                    "{role_label} PostScript/EPS parser evidence: lines={line_count} bounding-boxes={bounding_box_count} moveto={moveto_count} lineto={lineto_count} strokes={stroke_count} showpage={showpage_count}"
                ),
            }]
        }
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "drawing::postscript-unreadable".to_string(),
            message: format!("{role_label} PostScript/EPS sidecar could not be read: {error}"),
        }],
    }
}

fn hpgl_drawing_diagnostics(
    path: &std::path::Path,
    role_label: &str,
    source: &str,
) -> Vec<Diagnostic> {
    match std::fs::read_to_string(path) {
        Ok(text) => {
            let upper = text.to_ascii_uppercase();
            let pen_up_count = upper.matches("PU").count();
            let pen_down_count = upper.matches("PD").count();
            let absolute_count = upper.matches("PA").count();
            let relative_count = upper.matches("PR").count();
            let pen_select_count = upper.matches("SP").count();
            let circle_count = upper.matches("CI").count();
            let label_count = upper.matches("LB").count();
            vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "drawing::hpgl-summary".to_string(),
                message: format!(
                    "{role_label} HPGL/plot parser evidence: pen-up={pen_up_count} pen-down={pen_down_count} absolute={absolute_count} relative={relative_count} pen-select={pen_select_count} circles={circle_count} labels={label_count}"
                ),
            }]
        }
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "drawing::hpgl-unreadable".to_string(),
            message: format!("{role_label} HPGL/plot sidecar could not be read: {error}"),
        }],
    }
}

fn raster_drawing_diagnostics(
    path: &std::path::Path,
    role_label: &str,
    source: &str,
    extension: &str,
) -> Vec<Diagnostic> {
    match std::fs::read(path) {
        Ok(bytes) => match image_header_dimensions(&bytes) {
            Some((width, height)) => vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "drawing::raster-summary".to_string(),
                message: format!(
                    "{role_label} {} parser evidence: {}; raster drawings are retained for visual review and not imported as dimensional geometry",
                    review_image_label(extension),
                    image_dimension_evidence(width, height)
                ),
            }],
            None => vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "drawing::raster-unknown-dimensions".to_string(),
                message: format!("{role_label} raster drawing dimensions could not be read"),
            }],
        },
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "drawing::raster-unreadable".to_string(),
            message: format!("{role_label} raster drawing sidecar could not be read: {error}"),
        }],
    }
}

fn svg_drawing_diagnostics(
    path: &std::path::Path,
    role_label: &str,
    source: &str,
) -> Vec<Diagnostic> {
    match std::fs::read_to_string(path) {
        Ok(text) => match svg_drawing_summary(&text) {
            Ok(summary) => {
                let root_name = summary.root_name.as_deref().unwrap_or("<missing>");
                let geometry_count = summary.geometry_count();
                let mut diagnostics = vec![Diagnostic {
                    source: source.to_string(),
                    line: None,
                    severity: Severity::Warning,
                    code: "drawing::svg-summary".to_string(),
                    message: format!(
                        "{role_label} SVG parsed as root={root_name:?} geometry={geometry_count} paths={} rects={} circles={} ellipses={} lines={} polylines={} polygons={} text={} images={} uses={} groups={} width={} height={} viewBox={} ids={} classes={} style-attrs={} transforms={} hrefs={}",
                        summary.path_count,
                        summary.rect_count,
                        summary.circle_count,
                        summary.ellipse_count,
                        summary.line_count,
                        summary.polyline_count,
                        summary.polygon_count,
                        summary.text_count,
                        summary.image_count,
                        summary.use_count,
                        summary.group_count,
                        summary.width.as_deref().unwrap_or("<missing>"),
                        summary.height.as_deref().unwrap_or("<missing>"),
                        summary.view_box.as_deref().unwrap_or("<missing>"),
                        summary.id_count,
                        summary.class_count,
                        summary.style_attr_count,
                        summary.transform_count,
                        summary.href_count
                    ),
                }];
                if geometry_count == 0 {
                    diagnostics.push(Diagnostic {
                        source: source.to_string(),
                        line: None,
                        severity: Severity::Warning,
                        code: "drawing::svg-no-geometry".to_string(),
                        message: format!(
                            "{role_label} SVG contains no common drawable geometry elements"
                        ),
                    });
                }
                diagnostics
            }
            Err(error) => vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "drawing::svg-invalid".to_string(),
                message: format!("{role_label} SVG could not be parsed: {error}"),
            }],
        },
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "drawing::svg-unreadable".to_string(),
            message: format!("{role_label} SVG sidecar could not be read: {error}"),
        }],
    }
}

#[derive(Default)]
struct SvgDrawingSummary {
    root_name: Option<String>,
    path_count: usize,
    rect_count: usize,
    circle_count: usize,
    ellipse_count: usize,
    line_count: usize,
    polyline_count: usize,
    polygon_count: usize,
    text_count: usize,
    image_count: usize,
    use_count: usize,
    group_count: usize,
    width: Option<String>,
    height: Option<String>,
    view_box: Option<String>,
    id_count: usize,
    class_count: usize,
    style_attr_count: usize,
    transform_count: usize,
    href_count: usize,
}

impl SvgDrawingSummary {
    fn geometry_count(&self) -> usize {
        self.path_count
            + self.rect_count
            + self.circle_count
            + self.ellipse_count
            + self.line_count
            + self.polyline_count
            + self.polygon_count
    }
}

fn svg_drawing_summary(text: &str) -> Result<SvgDrawingSummary, quick_xml::Error> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader as XmlReader;

    let mut summary = SvgDrawingSummary::default();
    let mut reader = XmlReader::from_str(text);
    reader.config_mut().trim_text(true);
    loop {
        match reader.read_event()? {
            Event::Start(element) | Event::Empty(element) => {
                let name = String::from_utf8_lossy(element.local_name().as_ref()).to_string();
                if summary.root_name.is_none() {
                    summary.root_name = Some(name.clone());
                }
                for attribute in element.attributes().flatten() {
                    let key = String::from_utf8_lossy(attribute.key.as_ref()).to_string();
                    let value = String::from_utf8_lossy(attribute.value.as_ref()).to_string();
                    if name == "svg" {
                        match key.as_str() {
                            "width" => summary.width = Some(value.clone()),
                            "height" => summary.height = Some(value.clone()),
                            "viewBox" => summary.view_box = Some(value.clone()),
                            _ => {}
                        }
                    }
                    match key.as_str() {
                        "id" => summary.id_count += 1,
                        "class" => summary.class_count += 1,
                        "style" => summary.style_attr_count += 1,
                        "transform" => summary.transform_count += 1,
                        _ if key == "href" || key.ends_with(":href") => summary.href_count += 1,
                        _ => {}
                    }
                }
                match name.as_str() {
                    "path" => summary.path_count += 1,
                    "rect" => summary.rect_count += 1,
                    "circle" => summary.circle_count += 1,
                    "ellipse" => summary.ellipse_count += 1,
                    "line" => summary.line_count += 1,
                    "polyline" => summary.polyline_count += 1,
                    "polygon" => summary.polygon_count += 1,
                    "text" => summary.text_count += 1,
                    "image" => summary.image_count += 1,
                    "use" => summary.use_count += 1,
                    "g" => summary.group_count += 1,
                    _ => {}
                }
            }
            Event::Eof => break,
            _ => {}
        }
    }
    Ok(summary)
}

fn dxf_drawing_diagnostics(
    path: &std::path::Path,
    role_label: &str,
    source: &str,
) -> Vec<Diagnostic> {
    match std::fs::read_to_string(path) {
        Ok(text) => {
            let summary = dxf_drawing_summary(&text);
            let entity_count = summary.entity_count();
            let mut diagnostics = vec![Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: "drawing::dxf-summary".to_string(),
                message: format!(
                    "{role_label} DXF parsed as sections={} entities-section={} entities={} lines={} lwpolylines={} polylines={} circles={} arcs={} text={} mtext={} dimensions={} inserts={} layers={} colors={} coordinate-pairs={} bounds={}",
                    summary.section_count,
                    summary.entities_section_count,
                    entity_count,
                    summary.line_count,
                    summary.lwpolyline_count,
                    summary.polyline_count,
                    summary.circle_count,
                    summary.arc_count,
                    summary.text_count,
                    summary.mtext_count,
                    summary.dimension_count,
                    summary.insert_count,
                    summary.layer_names.len(),
                    summary.color_count,
                    summary.coordinate_pair_count,
                    summary
                        .bounds_string()
                        .unwrap_or_else(|| "<missing>".to_string())
                ),
            }];
            if summary.entities_section_count == 0 || entity_count == 0 {
                diagnostics.push(Diagnostic {
                    source: source.to_string(),
                    line: None,
                    severity: Severity::Warning,
                    code: "drawing::dxf-no-entities".to_string(),
                    message: format!("{role_label} DXF contains no common drawing entities"),
                });
            }
            diagnostics
        }
        Err(error) => vec![Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: "drawing::dxf-unreadable".to_string(),
            message: format!("{role_label} DXF sidecar could not be read: {error}"),
        }],
    }
}

#[derive(Default)]
struct DxfDrawingSummary {
    section_count: usize,
    entities_section_count: usize,
    line_count: usize,
    lwpolyline_count: usize,
    polyline_count: usize,
    circle_count: usize,
    arc_count: usize,
    text_count: usize,
    mtext_count: usize,
    dimension_count: usize,
    insert_count: usize,
    layer_names: std::collections::BTreeSet<String>,
    color_count: usize,
    coordinate_pair_count: usize,
    min_x: Option<f64>,
    min_y: Option<f64>,
    max_x: Option<f64>,
    max_y: Option<f64>,
}

impl DxfDrawingSummary {
    fn entity_count(&self) -> usize {
        self.line_count
            + self.lwpolyline_count
            + self.polyline_count
            + self.circle_count
            + self.arc_count
            + self.text_count
            + self.mtext_count
            + self.dimension_count
            + self.insert_count
    }

    fn bounds_string(&self) -> Option<String> {
        Some(format!(
            "{:.6},{:.6}..{:.6},{:.6}",
            self.min_x?, self.min_y?, self.max_x?, self.max_y?
        ))
    }
}

fn dxf_drawing_summary(text: &str) -> DxfDrawingSummary {
    let mut summary = DxfDrawingSummary::default();
    let mut in_entity_section = false;
    let mut pending_section = false;
    let mut pending_x = None;
    let lines: Vec<&str> = text.lines().collect();
    let mut index = 0;
    while index + 1 < lines.len() {
        let code = lines[index].trim();
        let value = lines[index + 1].trim();
        let upper = value.to_ascii_uppercase();
        match code {
            "0" if upper == "SECTION" => {
                summary.section_count += 1;
                pending_section = true;
            }
            "2" if pending_section && upper == "ENTITIES" => {
                summary.entities_section_count += 1;
                in_entity_section = true;
                pending_section = false;
            }
            "0" if upper == "ENDSEC" => {
                in_entity_section = false;
                pending_section = false;
            }
            "0" if in_entity_section => match upper.as_str() {
                "LINE" => summary.line_count += 1,
                "LWPOLYLINE" => summary.lwpolyline_count += 1,
                "POLYLINE" => summary.polyline_count += 1,
                "CIRCLE" => summary.circle_count += 1,
                "ARC" => summary.arc_count += 1,
                "TEXT" => summary.text_count += 1,
                "MTEXT" => summary.mtext_count += 1,
                "DIMENSION" => summary.dimension_count += 1,
                "INSERT" => summary.insert_count += 1,
                _ => {}
            },
            "8" if in_entity_section && !value.is_empty() => {
                summary.layer_names.insert(value.to_string());
            }
            "62" if in_entity_section => summary.color_count += 1,
            "10" | "11" | "12" | "13" | "14" | "15" | "16" | "17" | "18" if in_entity_section => {
                pending_x = value.parse::<f64>().ok();
            }
            "20" | "21" | "22" | "23" | "24" | "25" | "26" | "27" | "28" if in_entity_section => {
                if let (Some(x), Ok(y)) = (pending_x.take(), value.parse::<f64>()) {
                    summary.coordinate_pair_count += 1;
                    summary.min_x = Some(summary.min_x.map_or(x, |min| min.min(x)));
                    summary.max_x = Some(summary.max_x.map_or(x, |max| max.max(x)));
                    summary.min_y = Some(summary.min_y.map_or(y, |min| min.min(y)));
                    summary.max_y = Some(summary.max_y.map_or(y, |max| max.max(y)));
                }
            }
            _ => {}
        }
        index += 2;
    }
    summary
}

fn text_artifact_kind_diagnostics(
    files: &[io::DiscoveredFile],
    kind: checks::TextArtifactParserKind,
) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();
    for file in files {
        match load_text_artifact(&file.path) {
            Ok(text) => {
                diagnostics.extend(checks::text_artifact_parser_diagnostics(
                    &checks::TextArtifact {
                        path: file.path.display().to_string(),
                        text,
                    },
                    kind,
                ));
                if is_spreadsheet_path(&file.path) {
                    diagnostics.extend(spreadsheet_parser_diagnostics(&file.path, kind));
                }
            }
            Err(error) => diagnostics.push(Diagnostic {
                source: file.path.display().to_string(),
                line: None,
                severity: Severity::Warning,
                code: format!("{}::unreadable", text_artifact_kind_code_prefix(kind)),
                message: format!(
                    "{} sidecar could not be read: {error}",
                    text_artifact_kind_label(kind)
                ),
            }),
        }
    }
    diagnostics
}

fn spreadsheet_parser_diagnostics(
    path: &std::path::Path,
    kind: checks::TextArtifactParserKind,
) -> Vec<Diagnostic> {
    use calamine::{Reader, SheetType, SheetVisible, Sheets, open_workbook_auto};

    let source = path.display().to_string();
    let mut workbook = match open_workbook_auto(path) {
        Ok(workbook) => workbook,
        Err(error) => {
            return vec![Diagnostic {
                source,
                line: None,
                severity: Severity::Warning,
                code: format!(
                    "{}::spreadsheet-unreadable",
                    text_artifact_kind_code_prefix(kind)
                ),
                message: format!(
                    "{} spreadsheet could not be opened for workbook diagnostics: {error}",
                    text_artifact_kind_label(kind)
                ),
            }];
        }
    };

    let mut diagnostics = Vec::new();
    let sheet_metadata = workbook.sheets_metadata();
    let hidden_sheet_count = sheet_metadata
        .iter()
        .filter(|sheet| {
            matches!(
                sheet.visible,
                SheetVisible::Hidden | SheetVisible::VeryHidden
            )
        })
        .count();
    if hidden_sheet_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.clone(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-hidden-sheets",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {hidden_sheet_count} hidden sheet(s); table extraction may omit review-only or supplier-only data",
                text_artifact_kind_label(kind)
            ),
        });
    }

    let non_worksheet_count = sheet_metadata
        .iter()
        .filter(|sheet| sheet.typ != SheetType::WorkSheet)
        .count();
    if non_worksheet_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.clone(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-non-worksheet-sheets",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {non_worksheet_count} non-worksheet sheet(s); chart, dialog, macro, and VBA sheet semantics are not converted into artifact rows",
                text_artifact_kind_label(kind)
            ),
        });
    }

    let defined_name_count = workbook.defined_names().len();
    if defined_name_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.clone(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-defined-names",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {defined_name_count} defined name(s); table extraction preserves cell values, not named-range semantics",
                text_artifact_kind_label(kind)
            ),
        });
    }

    match workbook.vba_project() {
        Ok(Some(_)) => diagnostics.push(Diagnostic {
            source: source.clone(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-vba-project",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains a VBA/macro project; macros are not executed during sidecar extraction",
                text_artifact_kind_label(kind)
            ),
        }),
        Ok(None) => {}
        Err(error) => diagnostics.push(Diagnostic {
            source: source.clone(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-vba-unreadable",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet VBA/macro project could not be inspected: {error}",
                text_artifact_kind_label(kind)
            ),
        }),
    }

    diagnostics.extend(xlsx_metadata_diagnostics(path, kind, &source));

    if let Sheets::Xlsx(xlsx) = &mut workbook {
        match xlsx.load_merged_regions() {
            Ok(()) => {
                let merged_region_count = xlsx.merged_regions().len();
                if merged_region_count > 0 {
                    diagnostics.push(Diagnostic {
                        source: source.clone(),
                        line: None,
                        severity: Severity::Warning,
                        code: format!(
                            "{}::spreadsheet-merged-regions",
                            text_artifact_kind_code_prefix(kind)
                        ),
                        message: format!(
                            "{} spreadsheet contains {merged_region_count} merged cell region(s); table extraction preserves cell values, not merged-header layout semantics",
                            text_artifact_kind_label(kind)
                        ),
                    });
                }
            }
            Err(error) => diagnostics.push(Diagnostic {
                source: source.clone(),
                line: None,
                severity: Severity::Warning,
                code: format!(
                    "{}::spreadsheet-merged-regions-unreadable",
                    text_artifact_kind_code_prefix(kind)
                ),
                message: format!(
                    "{} spreadsheet merged cell regions could not be inspected: {error}",
                    text_artifact_kind_label(kind)
                ),
            }),
        }
        match xlsx.load_tables() {
            Ok(()) => {
                let table_count = xlsx.table_names().len();
                if table_count > 0 {
                    diagnostics.push(Diagnostic {
                        source: source.clone(),
                        line: None,
                        severity: Severity::Warning,
                        code: format!(
                            "{}::spreadsheet-structured-tables",
                            text_artifact_kind_code_prefix(kind)
                        ),
                        message: format!(
                            "{} spreadsheet contains {table_count} native Excel table(s); extraction flattens visible cell values and does not preserve table range/header semantics",
                            text_artifact_kind_label(kind)
                        ),
                    });
                }
            }
            Err(error) => diagnostics.push(Diagnostic {
                source: source.clone(),
                line: None,
                severity: Severity::Warning,
                code: format!(
                    "{}::spreadsheet-structured-tables-unreadable",
                    text_artifact_kind_code_prefix(kind)
                ),
                message: format!(
                    "{} spreadsheet native Excel table metadata could not be inspected: {error}",
                    text_artifact_kind_label(kind)
                ),
            }),
        }
    }

    let sheet_names = workbook.sheet_names();
    let mut formula_count = 0usize;
    for sheet_name in &sheet_names {
        if let Ok(formulas) = workbook.worksheet_formula(sheet_name) {
            formula_count += formulas
                .rows()
                .flat_map(|row| row.iter())
                .filter(|formula| !formula.trim().is_empty())
                .count();
        }
    }
    if formula_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.clone(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-formulas",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {formula_count} formula cell(s); table extraction uses cached/displayed cell values and does not recalculate formulas",
                text_artifact_kind_label(kind)
            ),
        });
    }

    let mut error_cell_count = 0usize;
    let mut datetime_cell_count = 0usize;
    let mut populated_sheet_count = 0usize;
    let mut empty_sheet_count = 0usize;
    for (_sheet_name, range) in workbook.worksheets() {
        if range.rows().any(|row| {
            row.iter()
                .any(|cell| !matches!(cell, calamine::Data::Empty))
        }) {
            populated_sheet_count += 1;
        } else {
            empty_sheet_count += 1;
        }
        for cell in range.rows().flat_map(|row| row.iter()) {
            match cell {
                calamine::Data::Error(_) => error_cell_count += 1,
                calamine::Data::DateTime(_)
                | calamine::Data::DateTimeIso(_)
                | calamine::Data::DurationIso(_) => datetime_cell_count += 1,
                _ => {}
            }
        }
    }
    if populated_sheet_count > 1 {
        diagnostics.push(Diagnostic {
            source: source.clone(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-multiple-populated-sheets",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {populated_sheet_count} populated worksheet(s); sidecar extraction uses workbook sheet ordering and may include auxiliary tables",
                text_artifact_kind_label(kind)
            ),
        });
    }
    if empty_sheet_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.clone(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-empty-sheets",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {empty_sheet_count} empty worksheet(s); confirm the intended artifact table is present in the release package",
                text_artifact_kind_label(kind)
            ),
        });
    }
    if error_cell_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.clone(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-error-cells",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {error_cell_count} cell error value(s); review exported sidecar data before release",
                text_artifact_kind_label(kind)
            ),
        });
    }
    if datetime_cell_count > 0 {
        diagnostics.push(Diagnostic {
            source,
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-date-cells",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {datetime_cell_count} date/time cell(s); extracted text uses workbook display conversion",
                text_artifact_kind_label(kind)
            ),
        });
    }

    diagnostics
}

fn xlsx_metadata_diagnostics(
    path: &std::path::Path,
    kind: checks::TextArtifactParserKind,
    source: &str,
) -> Vec<Diagnostic> {
    use std::io::Read as _;

    let extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    if !matches!(extension.as_str(), "xlsx" | "xlsm" | "xlam") {
        return Vec::new();
    }

    let mut diagnostics = Vec::new();
    let file = match std::fs::File::open(path) {
        Ok(file) => file,
        Err(error) => {
            diagnostics.push(Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: format!(
                    "{}::spreadsheet-xlsx-metadata-unreadable",
                    text_artifact_kind_code_prefix(kind)
                ),
                message: format!(
                    "{} spreadsheet XLSX metadata could not be opened: {error}",
                    text_artifact_kind_label(kind)
                ),
            });
            return diagnostics;
        }
    };
    let mut archive = match zip::ZipArchive::new(file) {
        Ok(archive) => archive,
        Err(error) => {
            diagnostics.push(Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: format!(
                    "{}::spreadsheet-xlsx-metadata-unreadable",
                    text_artifact_kind_code_prefix(kind)
                ),
                message: format!(
                    "{} spreadsheet XLSX metadata could not be inspected as a ZIP package: {error}",
                    text_artifact_kind_label(kind)
                ),
            });
            return diagnostics;
        }
    };

    if let Ok(mut styles) = archive.by_name("xl/styles.xml") {
        let mut xml = String::new();
        match styles.read_to_string(&mut xml) {
            Ok(_) => {
                match count_xml_elements(&xml, &[b"numFmt", b"xf", b"fill", b"border", b"dxf"]) {
                    Ok(counts) => {
                        let custom_number_format_count = counts[0];
                        let cell_format_count = counts[1];
                        let fill_count = counts[2];
                        let border_count = counts[3];
                        let differential_format_count = counts[4];
                        if custom_number_format_count > 0 {
                            diagnostics.push(Diagnostic {
                            source: source.to_string(),
                            line: None,
                            severity: Severity::Warning,
                            code: format!(
                                "{}::spreadsheet-custom-number-formats",
                                text_artifact_kind_code_prefix(kind)
                            ),
                            message: format!(
                                "{} spreadsheet contains {custom_number_format_count} custom number format(s); extraction preserves converted cell values, not workbook display-format intent",
                                text_artifact_kind_label(kind)
                            ),
                        });
                        }
                        if cell_format_count > 1
                            || fill_count > 2
                            || border_count > 1
                            || differential_format_count > 0
                        {
                            diagnostics.push(Diagnostic {
                            source: source.to_string(),
                            line: None,
                            severity: Severity::Warning,
                            code: format!(
                                "{}::spreadsheet-cell-styles",
                                text_artifact_kind_code_prefix(kind)
                            ),
                            message: format!(
                                "{} spreadsheet contains workbook cell style metadata; extraction preserves cell values, not color, border, or conditional style semantics",
                                text_artifact_kind_label(kind)
                            ),
                        });
                        }
                    }
                    Err(error) => diagnostics.push(Diagnostic {
                        source: source.to_string(),
                        line: None,
                        severity: Severity::Warning,
                        code: format!(
                            "{}::spreadsheet-xlsx-metadata-unreadable",
                            text_artifact_kind_code_prefix(kind)
                        ),
                        message: format!(
                            "{} spreadsheet style metadata could not be parsed: {error}",
                            text_artifact_kind_label(kind)
                        ),
                    }),
                }
            }
            Err(error) => diagnostics.push(Diagnostic {
                source: source.to_string(),
                line: None,
                severity: Severity::Warning,
                code: format!(
                    "{}::spreadsheet-xlsx-metadata-unreadable",
                    text_artifact_kind_code_prefix(kind)
                ),
                message: format!(
                    "{} spreadsheet style metadata could not be read: {error}",
                    text_artifact_kind_label(kind)
                ),
            }),
        }
    }

    let worksheet_names: Vec<String> = archive
        .file_names()
        .filter(|name| name.starts_with("xl/worksheets/sheet") && name.ends_with(".xml"))
        .map(ToOwned::to_owned)
        .collect();
    let drawing_part_count = archive
        .file_names()
        .filter(|name| name.starts_with("xl/drawings/") && name.ends_with(".xml"))
        .count();
    let embedded_media_count = archive
        .file_names()
        .filter(|name| name.starts_with("xl/media/"))
        .count();
    let comment_part_count = archive
        .file_names()
        .filter(|name| {
            (name.starts_with("xl/comments") && name.ends_with(".xml"))
                || name.starts_with("xl/threadedComments/")
        })
        .count();
    let chart_part_count = archive
        .file_names()
        .filter(|name| name.starts_with("xl/charts/") && name.ends_with(".xml"))
        .count();
    let pivot_table_part_count = archive
        .file_names()
        .filter(|name| name.starts_with("xl/pivotTables/") && name.ends_with(".xml"))
        .count();
    if drawing_part_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-drawing-objects",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {drawing_part_count} drawing object part(s); extraction preserves cell values, not shapes, callouts, or layout annotations",
                text_artifact_kind_label(kind)
            ),
        });
    }
    if embedded_media_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-embedded-media",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {embedded_media_count} embedded media object(s); extraction does not preserve images or visual review evidence",
                text_artifact_kind_label(kind)
            ),
        });
    }
    if comment_part_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-comments",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {comment_part_count} comment/threaded-comment part(s); extraction preserves cell values, not reviewer comments",
                text_artifact_kind_label(kind)
            ),
        });
    }
    if chart_part_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-charts",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {chart_part_count} chart part(s); extraction preserves source cell values, not charted review summaries",
                text_artifact_kind_label(kind)
            ),
        });
    }
    if pivot_table_part_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-pivot-tables",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {pivot_table_part_count} pivot table part(s); extraction preserves worksheet values, not pivot cache or aggregation semantics",
                text_artifact_kind_label(kind)
            ),
        });
    }

    let mut conditional_formatting_count = 0usize;
    let mut data_validation_count = 0usize;
    let mut autofilter_count = 0usize;
    let mut hyperlink_count = 0usize;
    for worksheet_name in worksheet_names {
        let mut xml = String::new();
        let Ok(mut worksheet) = archive.by_name(&worksheet_name) else {
            continue;
        };
        if worksheet.read_to_string(&mut xml).is_err() {
            continue;
        }
        if let Ok(counts) = count_xml_elements(
            &xml,
            &[
                b"conditionalFormatting",
                b"dataValidation",
                b"autoFilter",
                b"hyperlink",
            ],
        ) {
            conditional_formatting_count += counts[0];
            data_validation_count += counts[1];
            autofilter_count += counts[2];
            hyperlink_count += counts[3];
        }
    }
    if conditional_formatting_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-conditional-formatting",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {conditional_formatting_count} conditional formatting region(s); extraction preserves cell values, not style-driven review cues",
                text_artifact_kind_label(kind)
            ),
        });
    }
    if data_validation_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-data-validations",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {data_validation_count} data validation rule(s); extraction preserves existing values, not workbook input constraints",
                text_artifact_kind_label(kind)
            ),
        });
    }
    if autofilter_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-autofilters",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {autofilter_count} worksheet autofilter(s); extraction preserves rows, not filter visibility semantics",
                text_artifact_kind_label(kind)
            ),
        });
    }
    if hyperlink_count > 0 {
        diagnostics.push(Diagnostic {
            source: source.to_string(),
            line: None,
            severity: Severity::Warning,
            code: format!(
                "{}::spreadsheet-hyperlinks",
                text_artifact_kind_code_prefix(kind)
            ),
            message: format!(
                "{} spreadsheet contains {hyperlink_count} hyperlink(s); extraction preserves displayed cell values, not linked destinations",
                text_artifact_kind_label(kind)
            ),
        });
    }

    diagnostics
}

fn count_xml_elements(xml: &str, names: &[&[u8]]) -> Result<Vec<usize>, quick_xml::Error> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader as XmlReader;

    let mut counts = vec![0usize; names.len()];
    let mut reader = XmlReader::from_str(xml);
    reader.config_mut().trim_text(true);
    loop {
        match reader.read_event()? {
            Event::Start(element) | Event::Empty(element) => {
                let local_name = element.local_name();
                for (index, name) in names.iter().enumerate() {
                    if local_name.as_ref() == *name {
                        counts[index] += 1;
                    }
                }
            }
            Event::Eof => break,
            _ => {}
        }
    }
    Ok(counts)
}

fn text_artifact_kind_code_prefix(kind: checks::TextArtifactParserKind) -> &'static str {
    match kind {
        checks::TextArtifactParserKind::Bom => "artifact-table::bom",
        checks::TextArtifactParserKind::Centroid => "artifact-table::centroid",
        checks::TextArtifactParserKind::Netlist => "artifact-table::netlist",
    }
}

fn text_artifact_kind_label(kind: checks::TextArtifactParserKind) -> &'static str {
    match kind {
        checks::TextArtifactParserKind::Bom => "BOM",
        checks::TextArtifactParserKind::Centroid => "centroid",
        checks::TextArtifactParserKind::Netlist => "netlist",
    }
}

fn kicad_drc_report_diagnostics(path: &PathBuf) -> Vec<Diagnostic> {
    let source = path.display().to_string();
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(error) => {
            return vec![Diagnostic {
                source,
                line: None,
                severity: Severity::Warning,
                code: "kicad-drc::report-unreadable".to_string(),
                message: format!("KiCad DRC report could not be read: {error}"),
            }];
        }
    };
    let value: serde_json::Value = match serde_json::from_str(&text) {
        Ok(value) => value,
        Err(error) => {
            return vec![Diagnostic {
                source,
                line: None,
                severity: Severity::Warning,
                code: "kicad-drc::report-malformed-json".to_string(),
                message: format!("KiCad DRC JSON report could not be parsed: {error}"),
            }];
        }
    };

    let mut diagnostics = Vec::new();
    collect_kicad_drc_diagnostics(&source, &value, &mut diagnostics);
    diagnostics
}

fn collect_kicad_drc_diagnostics(
    source: &str,
    value: &serde_json::Value,
    diagnostics: &mut Vec<Diagnostic>,
) {
    match value {
        serde_json::Value::Array(items) => {
            for item in items {
                collect_kicad_drc_diagnostics(source, item, diagnostics);
            }
        }
        serde_json::Value::Object(object) => {
            if let Some(message) = kicad_drc_message(object) {
                diagnostics.push(Diagnostic {
                    source: source.to_string(),
                    line: None,
                    severity: kicad_drc_severity(object),
                    code: kicad_drc_code(object),
                    message,
                });
            }
            for child in object.values() {
                collect_kicad_drc_diagnostics(source, child, diagnostics);
            }
        }
        _ => {}
    }
}

fn kicad_drc_message(object: &serde_json::Map<String, serde_json::Value>) -> Option<String> {
    for key in ["description", "message", "text", "title", "error"] {
        if let Some(value) = object.get(key).and_then(|value| value.as_str()) {
            let trimmed = value.trim();
            if !trimmed.is_empty() {
                return Some(trimmed.to_string());
            }
        }
    }
    None
}

fn kicad_drc_code(object: &serde_json::Map<String, serde_json::Value>) -> String {
    for key in ["type", "code", "rule", "id"] {
        if let Some(value) = object.get(key).and_then(|value| value.as_str()) {
            let normalized = value
                .trim()
                .chars()
                .map(|character| {
                    if character.is_ascii_alphanumeric() {
                        character.to_ascii_lowercase()
                    } else {
                        '-'
                    }
                })
                .collect::<String>()
                .trim_matches('-')
                .to_string();
            if !normalized.is_empty() {
                return format!("kicad-drc::{normalized}");
            }
        }
    }
    "kicad-drc::violation".to_string()
}

fn kicad_drc_severity(object: &serde_json::Map<String, serde_json::Value>) -> Severity {
    let severity = object
        .get("severity")
        .or_else(|| object.get("level"))
        .and_then(|value| value.as_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    if severity.contains("warn") || severity.contains("exclusion") || severity.contains("ignore") {
        Severity::Warning
    } else {
        Severity::Error
    }
}

fn gerber_metadata_issue_code(
    kind: &crate::gerber_metadata::GerberMetadataIssueKind,
) -> &'static str {
    match kind {
        crate::gerber_metadata::GerberMetadataIssueKind::MissingFileAttributeValue { .. } => {
            "gerber::missing-file-attribute-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::DuplicateFileAttribute { .. } => {
            "gerber::duplicate-file-attribute"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::ConflictingFileAttribute { .. } => {
            "gerber::conflicting-file-attribute"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::InvalidFileAttributeValue { .. } => {
            "gerber::invalid-file-attribute-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::MissingApertureAttributeValue {
            ..
        } => "gerber::missing-aperture-attribute-value",
        crate::gerber_metadata::GerberMetadataIssueKind::InvalidApertureAttributeValue {
            ..
        } => "gerber::invalid-aperture-attribute-value",
        crate::gerber_metadata::GerberMetadataIssueKind::MissingApertureDefinitionValue {
            ..
        } => "gerber::missing-aperture-definition-value",
        crate::gerber_metadata::GerberMetadataIssueKind::InvalidApertureDefinitionValue {
            ..
        } => "gerber::invalid-aperture-definition-value",
        crate::gerber_metadata::GerberMetadataIssueKind::DuplicateApertureDefinition { .. } => {
            "gerber::duplicate-aperture-definition"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::ConflictingApertureDefinition {
            ..
        } => "gerber::conflicting-aperture-definition",
        crate::gerber_metadata::GerberMetadataIssueKind::MissingApertureMacroValue { .. } => {
            "gerber::missing-aperture-macro-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::InvalidApertureMacroValue { .. } => {
            "gerber::invalid-aperture-macro-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::DuplicateApertureMacro { .. } => {
            "gerber::duplicate-aperture-macro"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::ConflictingApertureMacro { .. } => {
            "gerber::conflicting-aperture-macro"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::UndefinedApertureSelection { .. } => {
            "gerber::undefined-aperture-selection"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::MissingCurrentAperture { .. } => {
            "gerber::missing-current-aperture"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::MissingPolarityCommandValue { .. } => {
            "gerber::missing-polarity-command-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::InvalidPolarityCommandValue { .. } => {
            "gerber::invalid-polarity-command-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::NestedRegion { .. } => {
            "gerber::nested-region"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::UnmatchedRegionEnd => {
            "gerber::unmatched-region-end"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::UnterminatedRegion { .. } => {
            "gerber::unterminated-region"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::InvalidStepRepeatCommandValue {
            ..
        } => "gerber::invalid-step-repeat-command-value",
        crate::gerber_metadata::GerberMetadataIssueKind::NestedStepRepeat { .. } => {
            "gerber::nested-step-repeat"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::UnmatchedStepRepeatEnd => {
            "gerber::unmatched-step-repeat-end"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::UnterminatedStepRepeat { .. } => {
            "gerber::unterminated-step-repeat"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::MissingObjectAttributeValue { .. } => {
            "gerber::missing-object-attribute-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::InvalidObjectAttributeValue { .. } => {
            "gerber::invalid-object-attribute-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::InvalidAttributeDeleteValue { .. } => {
            "gerber::invalid-attribute-delete-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::MissingImageCommandValue { .. } => {
            "gerber::missing-image-command-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::InvalidImageCommandValue { .. } => {
            "gerber::invalid-image-command-value"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::DuplicateImageCommand { .. } => {
            "gerber::duplicate-image-command"
        }
        crate::gerber_metadata::GerberMetadataIssueKind::ConflictingImageCommand { .. } => {
            "gerber::conflicting-image-command"
        }
    }
}

fn excellon_issue_code(kind: &excellon::ExcellonIssueKind) -> &'static str {
    match kind {
        excellon::ExcellonIssueKind::MissingUnitDeclaration => "excellon::missing-unit",
        excellon::ExcellonIssueKind::UnitConflict { .. } => "excellon::unit-conflict",
        excellon::ExcellonIssueKind::ZeroSuppressionDeclaration { .. } => {
            "excellon::zero-suppression"
        }
        excellon::ExcellonIssueKind::UnsupportedUnitDeclaration { .. } => {
            "excellon::unsupported-unit"
        }
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
        excellon::ExcellonIssueKind::RoutedSlotCommand { .. } => "excellon::routed-slot-command",
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

fn local_copper_density_window(min_width: f64) -> f64 {
    (min_width * LOCAL_COPPER_DENSITY_WINDOW_MULTIPLIER).max(1.0)
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
    use std::time::{Duration, Instant};
    use std::{fs, process};

    use clap::Parser;

    use crate::cli::{Check, Cli};
    use crate::config::{self, RuleOverrides};
    use crate::geometry::empty_sketch;
    use crate::io::{
        DiscoveredFile, IoAdapter, IoRole, SourceRecord, discover_gerber_dir,
        discover_package_sidecars, is_gerber_path,
    };
    use crate::kicad;

    use super::{
        Layer, PackageInputs, cli_with_inferred_layer_roles, explicit_layer_pairs,
        explicit_package_inputs, input_manifest, layer_pairs, load_all_layers, load_boards,
        load_excellon_drills, load_ipc356_points, load_layers, load_text_artifacts,
        load_text_artifacts_with_sheets, manifest_input, package_inputs, parser_diagnostics, run,
        should_fail_on_findings, validate_board_outline_role, validate_layer_index,
        validate_layer_indexes, validate_silk_layer_roles, waiver_governance_selected,
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
            gerber_image_setup: crate::gerber_metadata::GerberImageSetup::default(),
            gerber_metadata: crate::gerber_metadata::GerberLayerMetadata::default(),
            gerber_aperture_definitions: Vec::new(),
            gerber_aperture_macros: Vec::new(),
            gerber_aperture_uses: Vec::new(),
            gerber_coordinate_operations: Vec::new(),
            gerber_polarity_changes: Vec::new(),
            gerber_image_transforms: Vec::new(),
            gerber_region_events: Vec::new(),
            gerber_step_repeat_events: Vec::new(),
            gerber_interpolation_events: Vec::new(),
            gerber_quadrant_events: Vec::new(),
            gerber_object_metadata: Vec::new(),
            gerber_attribute_deletes: Vec::new(),
            gerber_metadata_issues: Vec::new(),
            sketch: empty_sketch(None),
        }
    }

    #[test]
    fn waiver_governance_selection_tracks_explicit_check() {
        assert!(waiver_governance_selected(&[Check::WaiverGovernance]));
        assert!(waiver_governance_selected(&[
            Check::CopperBalance,
            Check::WaiverGovernance
        ]));
        assert!(!waiver_governance_selected(&[Check::CopperBalance]));
    }

    #[test]
    fn cli_exit_status_policy_allows_report_only_runs() {
        assert!(!should_fail_on_findings(false, 0));
        assert!(should_fail_on_findings(false, 1));
        assert!(!should_fail_on_findings(true, 1));
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
                min_solder_mask_opening_area_ratio: None,
                max_solder_mask_opening_area_ratio: None,
                stencil_thickness: None,
                min_stencil_area_ratio: None,
                min_width: None,
                min_mask_width: None,
                min_solder_mask_annular_ring: None,
                min_silkscreen_text_height: None,
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

    struct ComplexProjectFixture {
        label: &'static str,
        zip_path: &'static str,
        board_path: &'static str,
        gerber_dir: Option<&'static str>,
        edge_cuts: Option<&'static str>,
    }

    const COMPLEX_PROJECT_FIXTURES: &[ComplexProjectFixture] = &[
        ComplexProjectFixture {
            label: "cparti-fpga",
            zip_path: "docs/CPArti FPGA dev board.zip",
            board_path: "CPArti FPGA dev board.kicad_pcb",
            gerber_dir: Some("gerbers"),
            edge_cuts: Some("CPArti FPGA dev board-Edge_Cuts.gbr"),
        },
        ComplexProjectFixture {
            label: "hvp109a",
            zip_path: "docs/HVP109A.zip",
            board_path: "HVP109A.kicad_pcb",
            gerber_dir: None,
            edge_cuts: None,
        },
    ];

    fn complex_zip_temp_root(label: &str) -> PathBuf {
        std::env::temp_dir().join(format!(
            "hyperdrc-complex-project-{label}-{}",
            std::process::id()
        ))
    }

    fn extract_complex_project_zip(root: &Path, label: &str, zip_path: &str) -> Option<PathBuf> {
        let zip_path = Path::new(env!("CARGO_MANIFEST_DIR")).join(zip_path);
        if !zip_path.exists() {
            eprintln!(
                "skipping complex project fixture test: missing {}",
                zip_path.display()
            );
            return None;
        }

        let package_dir = root.join(label);
        fs::create_dir_all(&package_dir).unwrap();
        let output = process::Command::new("unzip")
            .arg("-q")
            .arg("-o")
            .arg(&zip_path)
            .arg("-d")
            .arg(&package_dir)
            .output();
        match output {
            Ok(output) if output.status.success() => Some(package_dir),
            Ok(output) => {
                eprintln!(
                    "skipping complex project fixture test: unzip failed for {}: {}",
                    zip_path.display(),
                    String::from_utf8_lossy(&output.stderr)
                );
                None
            }
            Err(error) => {
                eprintln!(
                    "skipping complex project fixture test: cannot run unzip for {}: {error}",
                    zip_path.display()
                );
                None
            }
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
    fn inferred_layer_roles_fill_missing_cli_roles_from_filenames() {
        let cli = Cli::parse_from(["hyperdrc"]);
        let layers = [
            "board.gtl",
            "board.gbl",
            "board.gts",
            "board.gbs",
            "board.gtp",
            "board.gbp",
            "board.gto",
            "board.gbo",
            "outline.gko",
        ]
        .into_iter()
        .map(make_layer)
        .collect::<Vec<_>>();

        let cli = cli_with_inferred_layer_roles(cli, &layers);

        assert_eq!(cli.board_outline, Some(8));
        assert_eq!(cli.copper_layers, vec![0, 1]);
        assert_eq!(cli.mask_layers, vec![2, 3]);
        assert_eq!(cli.silk_layers, vec![6, 7]);
        assert_eq!(cli.paste_pairs, vec!["4:0", "5:1"]);
        assert_eq!(cli.mask_pairs, vec!["0:2", "1:3"]);
        assert_eq!(cli.silk_pairs, vec!["6:2", "7:3"]);
    }

    #[test]
    fn inferred_layer_roles_use_x2_file_function_and_preserve_explicit_cli_roles() {
        let cli = Cli::parse_from(["hyperdrc", "--copper-layer", "2"]);
        let mut layers = vec![
            make_layer("opaque-a.gbr"),
            make_layer("opaque-b.gbr"),
            make_layer("manual-copper.gbr"),
        ];
        layers[0].gerber_metadata.file_function = Some("Profile,NP".to_string());
        layers[1].gerber_metadata.file_function = Some("Soldermask,Top".to_string());
        layers[2].gerber_metadata.file_function = Some("Copper,L1,Top".to_string());

        let cli = cli_with_inferred_layer_roles(cli, &layers);

        assert_eq!(cli.board_outline, Some(0));
        assert_eq!(cli.mask_layers, vec![1]);
        assert_eq!(
            cli.copper_layers,
            vec![2],
            "explicit CLI copper selection should remain authoritative"
        );
    }

    #[test]
    fn run_rejects_empty_input_set() {
        let cli = Cli::parse_from(["hyperdrc"]);

        let error = run(cli).unwrap_err().to_string();

        assert!(error.contains("provide at least one"));
    }

    #[test]
    fn complex_project_zip_kicad_board_completes_smoke_check_suite() {
        let root = complex_zip_temp_root("kicad-smoke-suite");
        let _ = fs::remove_dir_all(&root);
        fs::create_dir_all(&root).unwrap();

        let started = Instant::now();
        for fixture in COMPLEX_PROJECT_FIXTURES {
            let Some(package_dir) =
                extract_complex_project_zip(&root, fixture.label, fixture.zip_path)
            else {
                continue;
            };
            let pcb_path = package_dir.join(fixture.board_path);
            assert!(
                pcb_path.exists(),
                "missing extracted board {}",
                pcb_path.display()
            );

            let cli = Cli::parse_from([
                "hyperdrc",
                "--kicad-pcb",
                pcb_path.to_str().unwrap(),
                "--check",
                "layer-sanity",
                "--check",
                "board-outline-sanity",
                "--check",
                "min-copper-neck",
                "--check",
                "drill-spacing",
                "--min-width",
                "0.0762",
                "--format",
                "text",
            ]);
            let outcome = run(cli).unwrap_or_else(|error| {
                panic!(
                    "smoke check suite failed for complex project board {}: {error}",
                    pcb_path.display()
                );
            });
            assert!(!outcome.report.inputs.is_empty());
        }

        let _ = fs::remove_dir_all(&root);
        assert!(
            started.elapsed() < Duration::from_secs(120),
            "complex project KiCad smoke suite took {:?}",
            started.elapsed()
        );
    }

    #[test]
    fn complex_project_gerber_package_completes_smoke_check_suite() {
        let root = complex_zip_temp_root("gerber-smoke-suite");
        let _ = fs::remove_dir_all(&root);
        fs::create_dir_all(&root).unwrap();

        let started = Instant::now();
        let mut covered_gerber_fixture = false;
        for fixture in COMPLEX_PROJECT_FIXTURES {
            let Some(gerber_dir_name) = fixture.gerber_dir else {
                continue;
            };
            let Some(edge_cuts_name) = fixture.edge_cuts else {
                continue;
            };
            let Some(package_dir) =
                extract_complex_project_zip(&root, fixture.label, fixture.zip_path)
            else {
                continue;
            };
            let gerber_dir = package_dir.join(gerber_dir_name);
            assert!(
                gerber_dir.exists(),
                "missing extracted Gerber directory {}",
                gerber_dir.display()
            );
            let smoke_dir = package_dir.join("gerber-smoke-subset");
            fs::create_dir_all(&smoke_dir).unwrap();
            fs::copy(
                gerber_dir.join(edge_cuts_name),
                smoke_dir.join(edge_cuts_name),
            )
            .unwrap();

            let cli = Cli::parse_from([
                "hyperdrc",
                "--gerber-dir",
                smoke_dir.to_str().unwrap(),
                "--check",
                "layer-sanity",
                "--min-width",
                "0.0762",
                "--format",
                "text",
            ]);
            let outcome = run(cli).unwrap_or_else(|error| {
                panic!(
                    "smoke check suite failed for complex project Gerbers in {}: {error}",
                    smoke_dir.display()
                );
            });
            assert!(!outcome.report.inputs.is_empty());
            covered_gerber_fixture = true;
        }
        let _ = fs::remove_dir_all(&root);

        assert!(covered_gerber_fixture);
        assert!(
            started.elapsed() < Duration::from_secs(120),
            "complex project Gerber smoke suite took {:?}",
            started.elapsed()
        );
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
        let excellon_report = crate::excellon::parse_excellon_report(
            "T01\nXbadY0200\nG85X010000Y010000X012000Y010000\n",
            Path::new("panel.drl"),
        );
        let ipc356_report =
            crate::ipc356::parse_ipc356_report("327 missing-coordinates\n", Path::new("board.ipc"));

        let diagnostics = parser_diagnostics(
            &[],
            &[excellon_report],
            &[ipc356_report],
            &[],
            &PackageInputs::default(),
        );

        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "excellon::unknown-tool-selection"
                || diagnostic.code == "excellon::invalid-coordinate"
        }));
        assert!(
            diagnostics
                .iter()
                .any(|diagnostic| diagnostic.code == "ipc356::malformed-test-record")
        );
        assert!(
            diagnostics
                .iter()
                .any(|diagnostic| diagnostic.code == "excellon::routed-slot-command")
        );
    }

    #[test]
    fn parser_diagnostics_collect_kicad_drc_report_entries() {
        let path =
            std::env::temp_dir().join(format!("hyperdrc-kicad-drc-{}.json", std::process::id()));
        std::fs::write(
            &path,
            r#"{
  "violations": [
    {
      "type": "clearance",
      "severity": "error",
      "description": "Copper clearance violation"
    },
    {
      "type": "silk",
      "severity": "warning",
      "message": "Silkscreen overlaps pad"
    }
  ]
}"#,
        )
        .unwrap();
        let conversion_output = crate::conversion::ConversionOutput {
            source_dir: PathBuf::from("board.kicad_pcb"),
            gerber_dir: PathBuf::from("converted"),
            drc_report: Some(path.clone()),
            input_hash: "hyperdrc-input-v1:test".to_string(),
            steps: Vec::new(),
            version: None,
            output_manifest: Vec::new(),
        };

        let diagnostics = parser_diagnostics(
            &[],
            &[],
            &[],
            &[conversion_output],
            &PackageInputs::default(),
        );

        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "kicad-drc::clearance"
                && diagnostic.severity == crate::report::Severity::Error
                && diagnostic.message == "Copper clearance violation"
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "kicad-drc::silk"
                && diagnostic.severity == crate::report::Severity::Warning
                && diagnostic.message == "Silkscreen overlaps pad"
        }));
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn parser_diagnostics_collect_converter_output_manifest_issues() {
        let dir =
            std::env::temp_dir().join(format!("hyperdrc-empty-conversion-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let conversion_output = crate::conversion::ConversionOutput {
            source_dir: PathBuf::from("board.kicad_pcb"),
            gerber_dir: dir.clone(),
            drc_report: None,
            input_hash: "hyperdrc-input-v1:test".to_string(),
            steps: Vec::new(),
            version: None,
            output_manifest: Vec::new(),
        };

        let diagnostics = parser_diagnostics(
            &[],
            &[],
            &[],
            &[conversion_output],
            &PackageInputs::default(),
        );

        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "converter::output-dir-empty"
                && diagnostic.message.contains("contains no Gerber layers")
        }));
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn parser_diagnostics_collect_manufacturing_handoff_issues() {
        let dir = std::env::temp_dir().join(format!(
            "hyperdrc-manufacturing-handoff-{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let ipc2581_path = dir.join("ipc2581.xml");
        let odb_path = dir.join("odb_output.zip");
        let gencad_path = dir.join("fixture_gencad.cad");
        let flying_probe_path = dir.join("flying_probe_report.csv");
        let boundary_scan_path = dir.join("boundary_scan.svf");
        let step_path = dir.join("board.step");
        let stl_path = dir.join("board.stl");
        let obj_path = dir.join("board.obj");
        let glb_path = dir.join("board.glb");
        let gltf_path = dir.join("board.gltf");
        let ply_path = dir.join("board.ply");
        let statistics_path = dir.join("statistics.json");
        let preview_png_path = dir.join("top_render.png");
        let preview_tiff_path = dir.join("bottom_preview.tiff");
        std::fs::write(
            &ipc2581_path,
            r#"<?xml version="1.0" encoding="UTF-8"?>
<IPC-2581>
  <Stackup units="mm"><StackupLayer name="L1"/></Stackup>
  <DielectricMaterial name="FR4" thickness="0.18"/>
  <Layer name="Top"/>
  <Net name="GND"/>
  <Component refdes="U1" packageRef="QFN"/>
  <Package name="QFN"/>
  <Drill diameter="0.30" x="1.0" y="2.0"/>
</IPC-2581>"#,
        )
        .unwrap();
        {
            use std::io::Write as _;

            let file = std::fs::File::create(&odb_path).unwrap();
            let mut zip = zip::ZipWriter::new(file);
            let options = zip::write::SimpleFileOptions::default();
            for name in [
                "matrix/matrix",
                "steps/pcb/profile",
                "steps/pcb/layers/top/features",
                "steps/pcb/layers/bottom/features",
                "steps/pcb/netlists/cadnet/netlist",
                "steps/pcb/eda/data",
                "steps/pcb/drill/tools",
            ] {
                zip.start_file(name, options).unwrap();
                let payload = match name {
                    "matrix/matrix" => "L TOP SIGNAL\nL BOTTOM SIGNAL\n",
                    "steps/pcb/profile" => "OB 0 0\nOS 10 0\nOS 10 5\n",
                    "steps/pcb/layers/top/features" => "$1 line\n$2 pad\n",
                    "steps/pcb/layers/bottom/features" => "$3 surface\n",
                    "steps/pcb/netlists/cadnet/netlist" => "NET GND\nNODE U1 1\n",
                    "steps/pcb/eda/data" => "CMP U1 QFN TOP\nPKG QFN\n",
                    "steps/pcb/drill/tools" => "T01 0.30\n",
                    _ => "",
                };
                zip.write_all(payload.as_bytes()).unwrap();
            }
            zip.finish().unwrap();
        }
        std::fs::write(
            &gencad_path,
            "$HEADER\n$BOARD\n$COMPONENTS\nU1 QFN 10.0 20.0 TOP\n$PARTS\nQFN MPN123\n$SIGNALS\nGND U1 1\n$ROUTES\nTRACK GND 0 0 1 1\n$TESTPOINTS\nTP1 GND U1 1 10.0 20.0\n$END\n",
        )
        .unwrap();
        std::fs::write(
            &flying_probe_path,
            "probe,net,refdes,x,y,result\nTP1,GND,U1.1,10.0,20.0,PASS\nTP2,VCC,U2.4,11.0,21.0,OPEN\n",
        )
        .unwrap();
        std::fs::write(
            &boundary_scan_path,
            "TRST OFF;\nSTATE RESET;\nSIR 8 TDI (AA) TDO (55) MASK (FF);\nSDR 32 TDI (00000000) SMASK (FFFFFFFF);\nRUNTEST 10 TCK;\n",
        )
        .unwrap();
        std::fs::write(
            &step_path,
            "ISO-10303-21;\nDATA;\n#1=PRODUCT('BOARD','BOARD','',());\n#2=PRODUCT_DEFINITION('','',#1,#3);\n#4=MANIFOLD_SOLID_BREP('',#5);\n#6=ADVANCED_FACE('',(),#7,.T.);\n#8=AXIS2_PLACEMENT_3D('',#9,#10,#11);\n#12=SI_UNIT(.MILLI.,.METRE.);\n#13=CLOSED_SHELL('',(#6));\n#14=SHAPE_REPRESENTATION('',(#4),#15);\n#16=LINE('',#9,#17);\n#18=DIMENSIONAL_SIZE(#4,'HEIGHT');\n#19=GEOMETRIC_TOLERANCE('POSITION','',#20);\nENDSEC;\nEND-ISO-10303-21;\n",
        )
        .unwrap();
        std::fs::write(
            &stl_path,
            "solid board\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid board\n",
        )
        .unwrap();
        std::fs::write(
            &obj_path,
            "o Board\nv 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1\n",
        )
        .unwrap();
        let mut glb = b"glTF".to_vec();
        glb.extend_from_slice(&2u32.to_le_bytes());
        glb.extend_from_slice(&12u32.to_le_bytes());
        std::fs::write(&glb_path, glb).unwrap();
        std::fs::write(
            &gltf_path,
            r#"{"asset":{"version":"2.0"},"scenes":[{}],"nodes":[{}],"meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}],"materials":[{}],"buffers":[{}],"bufferViews":[{}],"accessors":[{}],"images":[{}]}"#,
        )
        .unwrap();
        std::fs::write(
            &ply_path,
            "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2\n",
        )
        .unwrap();
        std::fs::write(
            &statistics_path,
            r#"{"board":{"layers":4,"tracks":12,"vias":3},"zones":[{"net":"GND","filled":true}],"notes":null}"#,
        )
        .unwrap();
        let mut png = b"\x89PNG\r\n\x1a\n\0\0\0\rIHDR".to_vec();
        png.extend_from_slice(&640u32.to_be_bytes());
        png.extend_from_slice(&480u32.to_be_bytes());
        png.extend_from_slice(&[8, 2, 0, 0, 0]);
        std::fs::write(&preview_png_path, png).unwrap();
        std::fs::write(
            &preview_tiff_path,
            [
                0x49, 0x49, 42, 0, 8, 0, 0, 0, 2, 0, 0, 1, 4, 0, 1, 0, 0, 0, 0, 4, 0, 0, 1, 1, 4,
                0, 1, 0, 0, 0, 0, 3, 0, 0,
            ],
        )
        .unwrap();
        let package_inputs = PackageInputs {
            manufacturing_handoff_files: vec![
                DiscoveredFile {
                    path: ipc2581_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &ipc2581_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: odb_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &odb_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: gencad_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &gencad_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: flying_probe_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &flying_probe_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: boundary_scan_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &boundary_scan_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: step_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &step_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: stl_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &stl_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: obj_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &obj_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: glb_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &glb_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: gltf_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &gltf_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: ply_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &ply_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: statistics_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &statistics_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: preview_png_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &preview_png_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
                DiscoveredFile {
                    path: preview_tiff_path.clone(),
                    source: SourceRecord::new(
                        IoAdapter::DirectFile,
                        IoRole::ManufacturingHandoff,
                        &preview_tiff_path,
                        Option::<&std::path::Path>::None,
                    ),
                },
            ],
            ..PackageInputs::default()
        };

        let diagnostics = parser_diagnostics(&[], &[], &[], &[], &package_inputs);

        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::ipc2581-present"
                && diagnostic.source == ipc2581_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::ipc2581-summary"
                && diagnostic.message.contains("layers=")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::ipc2581-design-evidence"
                && diagnostic.message.contains("named-layers=2")
                && diagnostic.message.contains("named-nets=1")
                && diagnostic.message.contains("refdes=1")
                && diagnostic.message.contains("package-names=1")
                && diagnostic.message.contains("drill-like=3")
                && diagnostic.message.contains("coordinate-like=2")
                && diagnostic.message.contains("unit-like=1")
                && diagnostic.message.contains("material-like=1")
                && diagnostic.message.contains("material-names=1")
                && diagnostic.message.contains("thickness-like=1")
                && diagnostic.message.contains("thickness-values=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::odb-package"
                && diagnostic.source == odb_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::odb-summary"
                && diagnostic.source == odb_path.display().to_string()
                && diagnostic.message.contains("entries=7")
                && diagnostic.message.contains("matrix=true")
                && diagnostic.message.contains("steps=1")
                && diagnostic.message.contains("feature-files=2")
                && diagnostic.message.contains("netlist-like=1")
                && diagnostic.message.contains("component-like=1")
                && diagnostic.message.contains("drill-tool-like=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::odb-content-summary"
                && diagnostic.source == odb_path.display().to_string()
                && diagnostic.message.contains("matrix-records=2")
                && diagnostic.message.contains("feature-records=3")
                && diagnostic.message.contains("profile-records=3")
                && diagnostic.message.contains("netlist-records=2")
                && diagnostic.message.contains("component-records=2")
                && diagnostic.message.contains("drill-tool-records=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::gencad-present"
                && diagnostic.source == gencad_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::gencad-summary"
                && diagnostic.message.contains("routes=1")
                && diagnostic.message.contains("component-records=1")
                && diagnostic.message.contains("part-records=1")
                && diagnostic.message.contains("signal-records=1")
                && diagnostic.message.contains("route-records=1")
                && diagnostic.message.contains("testpoint-records=1")
                && diagnostic.message.contains("unique-components=1")
                && diagnostic.message.contains("unique-parts=1")
                && diagnostic.message.contains("unique-signals=1")
                && diagnostic.message.contains("unique-testpoints=1")
                && diagnostic.message.contains("coordinate-like-records=3")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::test-inspection-present"
                && diagnostic.source == flying_probe_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::test-inspection-summary"
                && diagnostic.message.contains("pass/fail-like=2")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::test-inspection-table-summary"
                && diagnostic.source == flying_probe_path.display().to_string()
                && diagnostic.message.contains("rows=2")
                && diagnostic.message.contains("xy-columns=true")
                && diagnostic.message.contains("result-column=true")
                && diagnostic.message.contains("pass-results=1")
                && diagnostic.message.contains("open-results=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::boundary-scan-summary"
                && diagnostic.source == boundary_scan_path.display().to_string()
                && diagnostic.message.contains("commands=5")
                && diagnostic.message.contains("sir=1")
                && diagnostic.message.contains("sdr=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::mechanical-3d-present"
                && diagnostic.source == step_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::step-summary"
                && diagnostic.message.contains("products=2")
                && diagnostic.message.contains("solids=1")
                && diagnostic.message.contains("faces=1")
                && diagnostic.message.contains("shells=1")
                && diagnostic.message.contains("curves=1")
                && diagnostic.message.contains("shape-representations=1")
                && diagnostic.message.contains("dimensions=1")
                && diagnostic.message.contains("tolerances=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::mechanical-3d-present"
                && diagnostic.source == glb_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::stl-summary"
                && diagnostic.source == stl_path.display().to_string()
                && diagnostic.message.contains("ascii=true")
                && diagnostic.message.contains("facets=1")
                && diagnostic.message.contains("vertices=3")
                && diagnostic
                    .message
                    .contains("bounds=0.000000,0.000000,0.000000..1.000000,1.000000,0.000000")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::obj-summary"
                && diagnostic.source == obj_path.display().to_string()
                && diagnostic.message.contains("vertices=3")
                && diagnostic.message.contains("faces=1")
                && diagnostic
                    .message
                    .contains("bounds=0.000000,0.000000,0.000000..1.000000,1.000000,0.000000")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::glb-summary"
                && diagnostic.source == glb_path.display().to_string()
                && diagnostic.message.contains("version=2")
                && diagnostic.message.contains("declared-length=12")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::gltf-summary"
                && diagnostic.source == gltf_path.display().to_string()
                && diagnostic.message.contains("meshes=1")
                && diagnostic.message.contains("materials=1")
                && diagnostic.message.contains("buffer-views=1")
                && diagnostic.message.contains("accessors=1")
                && diagnostic.message.contains("primitives=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::mechanical-3d-present"
                && diagnostic.source == ply_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::ply-summary"
                && diagnostic.source == ply_path.display().to_string()
                && diagnostic.message.contains("vertices=3")
                && diagnostic.message.contains("faces=1")
                && diagnostic
                    .message
                    .contains("bounds=0.000000,0.000000,0.000000..1.000000,1.000000,0.000000")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::kicad-statistics-present"
                && diagnostic.source == statistics_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::kicad-statistics-summary"
                && diagnostic.message.contains("objects=3")
                && diagnostic.message.contains("numbers=3")
                && diagnostic.message.contains("booleans=1")
                && diagnostic.message.contains("nulls=1")
                && diagnostic.message.contains("layer-values=1")
                && diagnostic.message.contains("track-values=1")
                && diagnostic.message.contains("via-values=1")
                && diagnostic.message.contains("zone-arrays=1")
                && diagnostic.message.contains("filled-zone-values=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::review-image-present"
                && diagnostic.source == preview_png_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::review-image-summary"
                && diagnostic.source == preview_png_path.display().to_string()
                && diagnostic.message.contains("width=640")
                && diagnostic.message.contains("height=480")
                && diagnostic.message.contains("pixels=307200")
                && diagnostic.message.contains("aspect=1.333333")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "manufacturing-handoff::review-image-summary"
                && diagnostic.source == preview_tiff_path.display().to_string()
                && diagnostic.message.contains("width=1024")
                && diagnostic.message.contains("height=768")
                && diagnostic.message.contains("pixels=786432")
                && diagnostic.message.contains("aspect=1.333333")
        }));
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn parser_diagnostics_collect_text_sidecar_table_issues() {
        let process_id = std::process::id();
        let bom_path = std::env::temp_dir().join(format!("hyperdrc-bom-diag-{process_id}.csv"));
        let centroid_path =
            std::env::temp_dir().join(format!("hyperdrc-centroid-diag-{process_id}.csv"));
        std::fs::write(&bom_path, "Reference,Reference\nR1\n").unwrap();
        std::fs::write(
            &centroid_path,
            "Reference,X,Y,Rotation,Side\nU1,not-a-number,2.0,90,top\n",
        )
        .unwrap();
        let package_inputs = PackageInputs {
            bom_files: explicit_package_inputs(
                std::slice::from_ref(&bom_path),
                IoAdapter::DirectFile,
                IoRole::BomFile,
            ),
            centroid_files: explicit_package_inputs(
                std::slice::from_ref(&centroid_path),
                IoAdapter::DirectFile,
                IoRole::CentroidFile,
            ),
            ..PackageInputs::default()
        };

        let diagnostics = parser_diagnostics(&[], &[], &[], &[], &package_inputs);

        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "artifact-table::bom::duplicate-header"
                && diagnostic.source == bom_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "artifact-table::bom::ragged-row" && diagnostic.line == Some(2)
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "artifact-table::centroid::invalid-number"
                && diagnostic.line == Some(2)
                && diagnostic.message.contains("x coordinate")
        }));
        let _ = std::fs::remove_file(bom_path);
        let _ = std::fs::remove_file(centroid_path);
    }

    #[test]
    fn parser_diagnostics_collect_drawing_sidecar_parser_evidence() {
        let process_id = std::process::id();
        let svg_path = std::env::temp_dir().join(format!("hyperdrc-fab-drawing-{process_id}.svg"));
        let pdf_path = std::env::temp_dir().join(format!("hyperdrc-fab-drawing-{process_id}.pdf"));
        let dxf_path = std::env::temp_dir().join(format!("hyperdrc-panel-route-{process_id}.dxf"));
        let hpgl_path =
            std::env::temp_dir().join(format!("hyperdrc-panel-route-{process_id}.hpgl"));
        let dwg_path =
            std::env::temp_dir().join(format!("hyperdrc-assembly-fixture-{process_id}.dwg"));
        let eps_path =
            std::env::temp_dir().join(format!("hyperdrc-assembly-plot-{process_id}.eps"));
        let png_path =
            std::env::temp_dir().join(format!("hyperdrc-assembly-preview-{process_id}.png"));
        std::fs::write(
            &svg_path,
            r#"<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="80mm" viewBox="0 0 100 80"><g id="fab" class="notes" transform="translate(1 2)"><path style="stroke:black" d="M0 0H1V1Z"/><text>REV A</text></g></svg>"#,
        )
        .unwrap();
        std::fs::write(
            &pdf_path,
            b"%PDF-1.4\n1 0 obj\n<< /Type /Page /MediaBox [0 0 100 100] /Annots [] /Resources << /Font << /F1 2 0 R >> /XObject << /Im1 /Image >> >> >>\nstream\nBT (REV A) Tj ET\nendstream\nendobj\n",
        )
        .unwrap();
        std::fs::write(
            &dxf_path,
            "0\nSECTION\n2\nENTITIES\n0\nLWPOLYLINE\n8\nROUTE\n62\n3\n90\n2\n10\n0\n20\n0\n10\n10\n20\n5\n0\nCIRCLE\n8\nDRILL\n10\n1\n20\n2\n40\n0.5\n0\nENDSEC\n0\nEOF\n",
        )
        .unwrap();
        std::fs::write(&hpgl_path, "IN;SP1;PA0,0;PD100,0;PU;CI50;LBREV A\u{3};").unwrap();
        std::fs::write(&dwg_path, b"AC1027").unwrap();
        std::fs::write(
            &eps_path,
            "%!PS-Adobe-3.0 EPSF-3.0\n%%BoundingBox: 0 0 100 100\n0 0 moveto\n100 0 lineto\nstroke\nshowpage\n",
        )
        .unwrap();
        let mut png = b"\x89PNG\r\n\x1a\n\0\0\0\rIHDR".to_vec();
        png.extend_from_slice(&320u32.to_be_bytes());
        png.extend_from_slice(&240u32.to_be_bytes());
        png.extend_from_slice(&[8, 2, 0, 0, 0]);
        std::fs::write(&png_path, png).unwrap();
        let package_inputs = PackageInputs {
            fab_drawing_files: explicit_package_inputs(
                &[svg_path.clone(), pdf_path.clone()],
                IoAdapter::DirectFile,
                IoRole::FabDrawing,
            ),
            rout_drawing_files: explicit_package_inputs(
                &[dxf_path.clone(), hpgl_path.clone()],
                IoAdapter::DirectFile,
                IoRole::RoutDrawingFile,
            ),
            assembly_drawing_files: explicit_package_inputs(
                &[dwg_path.clone(), eps_path.clone(), png_path.clone()],
                IoAdapter::DirectFile,
                IoRole::AssemblyDrawing,
            ),
            ..PackageInputs::default()
        };

        let diagnostics = parser_diagnostics(&[], &[], &[], &[], &package_inputs);

        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "drawing::svg-summary"
                && diagnostic.source == svg_path.display().to_string()
                && diagnostic.message.contains("geometry=1")
                && diagnostic.message.contains("text=1")
                && diagnostic.message.contains("width=100mm")
                && diagnostic.message.contains("height=80mm")
                && diagnostic.message.contains("viewBox=0 0 100 80")
                && diagnostic.message.contains("ids=1")
                && diagnostic.message.contains("classes=1")
                && diagnostic.message.contains("style-attrs=1")
                && diagnostic.message.contains("transforms=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "drawing::dxf-summary"
                && diagnostic.source == dxf_path.display().to_string()
                && diagnostic.message.contains("entities=2")
                && diagnostic.message.contains("lwpolylines=1")
                && diagnostic.message.contains("circles=1")
                && diagnostic.message.contains("layers=2")
                && diagnostic.message.contains("colors=1")
                && diagnostic.message.contains("coordinate-pairs=3")
                && diagnostic
                    .message
                    .contains("bounds=0.000000,0.000000..10.000000,5.000000")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "drawing::binary-cad-present"
                && diagnostic.source == dwg_path.display().to_string()
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "drawing::pdf-summary"
                && diagnostic.source == pdf_path.display().to_string()
                && diagnostic.message.contains("page-markers=1")
                && diagnostic.message.contains("streams=2")
                && diagnostic.message.contains("fonts=1")
                && diagnostic.message.contains("xobjects=1")
                && diagnostic.message.contains("text-objects=1")
                && diagnostic.message.contains("text-shows=1")
                && diagnostic.message.contains("media-boxes=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "drawing::hpgl-summary"
                && diagnostic.source == hpgl_path.display().to_string()
                && diagnostic.message.contains("pen-down=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "drawing::postscript-summary"
                && diagnostic.source == eps_path.display().to_string()
                && diagnostic.message.contains("bounding-boxes=1")
        }));
        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "drawing::raster-summary"
                && diagnostic.source == png_path.display().to_string()
                && diagnostic.message.contains("width=320")
                && diagnostic.message.contains("height=240")
                && diagnostic.message.contains("pixels=76800")
                && diagnostic.message.contains("aspect=1.333333")
        }));
        let _ = std::fs::remove_file(svg_path);
        let _ = std::fs::remove_file(pdf_path);
        let _ = std::fs::remove_file(dxf_path);
        let _ = std::fs::remove_file(hpgl_path);
        let _ = std::fs::remove_file(dwg_path);
        let _ = std::fs::remove_file(eps_path);
        let _ = std::fs::remove_file(png_path);
    }

    #[test]
    fn parser_diagnostics_collect_spreadsheet_formula_warnings() {
        let path = std::env::temp_dir().join(format!(
            "hyperdrc-formula-bom-diag-{}.xlsx",
            std::process::id()
        ));
        write_formula_xlsx(&path);
        let package_inputs = PackageInputs {
            bom_files: explicit_package_inputs(
                std::slice::from_ref(&path),
                IoAdapter::DirectFile,
                IoRole::BomFile,
            ),
            ..PackageInputs::default()
        };

        let diagnostics = parser_diagnostics(&[], &[], &[], &[], &package_inputs);

        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "artifact-table::bom::spreadsheet-formulas"
                && diagnostic.message.contains("formula cell")
        }));
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn text_artifact_loader_preserves_workbook_formula_comments() {
        let path = std::env::temp_dir().join(format!(
            "hyperdrc-formula-comments-bom-{}.xlsx",
            std::process::id()
        ));
        write_formula_xlsx(&path);

        let artifacts = load_text_artifacts(std::slice::from_ref(&path)).unwrap();

        assert!(
            artifacts[0]
                .text
                .contains("# hyperdrc-workbook-formula sheet=Sheet1 cell=B2 formula=SUM(1,1)")
        );
        assert!(artifacts[0].text.contains("Reference\tQuantity\tMPN"));
        assert!(artifacts[0].text.contains("R1\t2\tRC0603"));

        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn parser_diagnostics_collect_workbook_semantics_warnings() {
        let path = std::env::temp_dir().join(format!(
            "hyperdrc-workbook-semantics-bom-diag-{}.xlsx",
            std::process::id()
        ));
        write_workbook_semantics_xlsx(&path);
        let package_inputs = PackageInputs {
            bom_files: explicit_package_inputs(
                std::slice::from_ref(&path),
                IoAdapter::DirectFile,
                IoRole::BomFile,
            ),
            ..PackageInputs::default()
        };

        let diagnostics = parser_diagnostics(&[], &[], &[], &[], &package_inputs);

        for code in [
            "artifact-table::bom::spreadsheet-hidden-sheets",
            "artifact-table::bom::spreadsheet-merged-regions",
            "artifact-table::bom::spreadsheet-structured-tables",
            "artifact-table::bom::spreadsheet-custom-number-formats",
            "artifact-table::bom::spreadsheet-cell-styles",
            "artifact-table::bom::spreadsheet-conditional-formatting",
            "artifact-table::bom::spreadsheet-data-validations",
            "artifact-table::bom::spreadsheet-autofilters",
            "artifact-table::bom::spreadsheet-hyperlinks",
            "artifact-table::bom::spreadsheet-drawing-objects",
            "artifact-table::bom::spreadsheet-embedded-media",
            "artifact-table::bom::spreadsheet-comments",
            "artifact-table::bom::spreadsheet-charts",
            "artifact-table::bom::spreadsheet-pivot-tables",
            "artifact-table::bom::spreadsheet-multiple-populated-sheets",
        ] {
            assert!(
                diagnostics.iter().any(|diagnostic| diagnostic.code == code),
                "missing diagnostic {code}: {diagnostics:#?}"
            );
        }
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn parser_diagnostics_collect_gerber_metadata_issues() {
        let mut layer = make_layer("board.gbr");
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 3,
                kind: crate::gerber_metadata::GerberMetadataIssueKind::ConflictingFileAttribute {
                    attribute: "TF.FileFunction".to_string(),
                    first: "Copper,L1,Top".to_string(),
                    duplicate: "Copper,L2,Inr".to_string(),
                },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 4,
                kind: crate::gerber_metadata::GerberMetadataIssueKind::InvalidFileAttributeValue {
                    attribute: "TF.FilePolarity".to_string(),
                    value: "Inverted".to_string(),
                },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 5,
                kind:
                    crate::gerber_metadata::GerberMetadataIssueKind::InvalidApertureAttributeValue {
                        attribute: "TA.AperFunction".to_string(),
                        value: "SMDPad".to_string(),
                    },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 6,
                kind:
                    crate::gerber_metadata::GerberMetadataIssueKind::InvalidObjectAttributeValue {
                        attribute: "TO.P".to_string(),
                        value: ",1".to_string(),
                    },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 7,
                kind: crate::gerber_metadata::GerberMetadataIssueKind::InvalidImageCommandValue {
                    command: "FS".to_string(),
                    value: "LAX35Y35".to_string(),
                },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
            line: 8,
            kind: crate::gerber_metadata::GerberMetadataIssueKind::InvalidApertureDefinitionValue {
                command: "ADD".to_string(),
                value: "9C,0.5".to_string(),
            },
        });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 9,
                kind: crate::gerber_metadata::GerberMetadataIssueKind::UndefinedApertureSelection {
                    d_code: 99,
                },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 10,
                kind: crate::gerber_metadata::GerberMetadataIssueKind::MissingCurrentAperture {
                    operation: "D03".to_string(),
                },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 11,
                kind:
                    crate::gerber_metadata::GerberMetadataIssueKind::InvalidPolarityCommandValue {
                        command: "LP".to_string(),
                        value: "X".to_string(),
                    },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 12,
                kind: crate::gerber_metadata::GerberMetadataIssueKind::UnterminatedRegion {
                    open_line: 12,
                },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 13,
                kind: crate::gerber_metadata::GerberMetadataIssueKind::UnterminatedStepRepeat {
                    open_line: 13,
                },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 14,
                kind: crate::gerber_metadata::GerberMetadataIssueKind::InvalidApertureMacroValue {
                    command: "AM".to_string(),
                    value: "BAD*99,1,2".to_string(),
                },
            });
        layer
            .gerber_metadata_issues
            .push(crate::gerber_metadata::GerberMetadataIssue {
                line: 15,
                kind:
                    crate::gerber_metadata::GerberMetadataIssueKind::InvalidAttributeDeleteValue {
                        command: "TD".to_string(),
                        value: "N".to_string(),
                    },
            });

        let diagnostics = parser_diagnostics(&[layer], &[], &[], &[], &PackageInputs::default());

        assert_eq!(diagnostics.len(), 13);
        assert_eq!(diagnostics[0].source, "board.gbr");
        assert_eq!(diagnostics[0].line, Some(3));
        assert_eq!(diagnostics[0].code, "gerber::conflicting-file-attribute");
        assert_eq!(diagnostics[1].line, Some(4));
        assert_eq!(diagnostics[1].code, "gerber::invalid-file-attribute-value");
        assert_eq!(diagnostics[2].line, Some(5));
        assert_eq!(
            diagnostics[2].code,
            "gerber::invalid-aperture-attribute-value"
        );
        assert_eq!(diagnostics[3].line, Some(6));
        assert_eq!(
            diagnostics[3].code,
            "gerber::invalid-object-attribute-value"
        );
        assert_eq!(diagnostics[4].line, Some(7));
        assert_eq!(diagnostics[4].code, "gerber::invalid-image-command-value");
        assert_eq!(diagnostics[5].line, Some(8));
        assert_eq!(
            diagnostics[5].code,
            "gerber::invalid-aperture-definition-value"
        );
        assert_eq!(diagnostics[6].line, Some(9));
        assert_eq!(diagnostics[6].code, "gerber::undefined-aperture-selection");
        assert_eq!(diagnostics[7].line, Some(10));
        assert_eq!(diagnostics[7].code, "gerber::missing-current-aperture");
        assert_eq!(diagnostics[8].line, Some(11));
        assert_eq!(
            diagnostics[8].code,
            "gerber::invalid-polarity-command-value"
        );
        assert_eq!(diagnostics[9].line, Some(12));
        assert_eq!(diagnostics[9].code, "gerber::unterminated-region");
        assert_eq!(diagnostics[10].line, Some(13));
        assert_eq!(diagnostics[10].code, "gerber::unterminated-step-repeat");
        assert_eq!(diagnostics[11].line, Some(14));
        assert_eq!(diagnostics[11].code, "gerber::invalid-aperture-macro-value");
        assert_eq!(diagnostics[12].line, Some(15));
        assert_eq!(
            diagnostics[12].code,
            "gerber::invalid-attribute-delete-value"
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
    fn text_artifact_loader_extracts_xlsx_tables() {
        let path =
            std::env::temp_dir().join(format!("hyperdrc-table-bom-{}.xlsx", std::process::id()));
        write_minimal_xlsx(
            &path,
            &[
                &["Reference", "Quantity", "MPN"],
                &["R1", "1", "RC0603"],
                &["C1", "2", "CC0603"],
            ],
        );

        let artifacts = load_text_artifacts(std::slice::from_ref(&path)).unwrap();

        assert!(artifacts[0].text.contains("Reference"));
        assert!(artifacts[0].text.contains("Quantity"));
        assert!(artifacts[0].text.contains("MPN"));
        assert!(artifacts[0].text.contains("R1"));
        assert!(artifacts[0].text.contains("1"));
        assert!(artifacts[0].text.contains("RC0603"));
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn text_artifact_loader_merges_workbook_sheets_with_matching_headers() {
        let path = std::env::temp_dir().join(format!(
            "hyperdrc-multisheet-bom-{}.xlsx",
            std::process::id()
        ));
        write_two_sheet_xlsx(
            &path,
            &[&["Reference", "Quantity", "MPN"], &["R1", "1", "RC0603"]],
            &[&["Reference", "Quantity", "MPN"], &["C1", "2", "CC0603"]],
        );

        let artifacts = load_text_artifacts(std::slice::from_ref(&path)).unwrap();

        assert!(artifacts[0].text.contains("R1\t1\tRC0603"));
        assert!(artifacts[0].text.contains("C1\t2\tCC0603"));
        assert_eq!(
            artifacts[0]
                .text
                .matches("Reference\tQuantity\tMPN")
                .count(),
            1
        );
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn text_artifact_loader_extracts_selected_workbook_sheet() {
        let path = std::env::temp_dir().join(format!(
            "hyperdrc-selected-sheet-bom-{}.xlsx",
            std::process::id()
        ));
        write_two_sheet_xlsx(
            &path,
            &[&["Reference", "Quantity", "MPN"], &["R1", "1", "RC0603"]],
            &[&["Designator", "Part"], &["C1", "CC0603"]],
        );

        let artifacts =
            load_text_artifacts_with_sheets(std::slice::from_ref(&path), &["Bottom".to_string()])
                .unwrap();

        assert!(artifacts[0].text.contains("Designator\tPart"));
        assert!(artifacts[0].text.contains("C1\tCC0603"));
        assert!(!artifacts[0].text.contains("Reference\tQuantity\tMPN"));
        assert!(!artifacts[0].text.contains("R1\t1\tRC0603"));
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn text_artifact_loader_merges_selected_workbook_sheets_with_different_headers() {
        let path = std::env::temp_dir().join(format!(
            "hyperdrc-selected-mixed-sheet-bom-{}.xlsx",
            std::process::id()
        ));
        write_two_sheet_xlsx(
            &path,
            &[&["Reference", "Quantity", "MPN"], &["R1", "1", "RC0603"]],
            &[&["Designator", "Variant", "DNP"], &["R1", "Proto", "No"]],
        );

        let artifacts = load_text_artifacts_with_sheets(
            std::slice::from_ref(&path),
            &["Top".to_string(), "Bottom".to_string()],
        )
        .unwrap();

        assert!(
            artifacts[0]
                .text
                .contains("Reference\tQuantity\tMPN\tVariant\tDNP")
        );
        assert!(artifacts[0].text.contains("R1\t1\tRC0603\t\t"));
        assert!(artifacts[0].text.contains("R1\t\t\tProto\tNo"));
        assert_eq!(artifacts[0].text.matches("Reference").count(), 1);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn text_artifact_loader_extracts_json_tables() {
        let path =
            std::env::temp_dir().join(format!("hyperdrc-table-bom-{}.json", std::process::id()));
        std::fs::write(
            &path,
            r#"{"components":[{"Reference":"R1","Quantity":1,"MPN":"RC0603"},{"Reference":"C1","Quantity":2,"MPN":"CC0603"}]}"#,
        )
        .unwrap();

        let artifacts = load_text_artifacts(std::slice::from_ref(&path)).unwrap();

        assert!(artifacts[0].text.contains("Reference"));
        assert!(artifacts[0].text.contains("Quantity"));
        assert!(artifacts[0].text.contains("MPN"));
        assert!(artifacts[0].text.contains("R1"));
        assert!(artifacts[0].text.contains("1"));
        assert!(artifacts[0].text.contains("RC0603"));
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn text_artifact_loader_extracts_nested_json_tables_and_flattened_fields() {
        let path = std::env::temp_dir().join(format!(
            "hyperdrc-nested-table-bom-{}.json",
            std::process::id()
        ));
        std::fs::write(
            &path,
            r#"{
  "project": {"name": "demo"},
  "release": {
    "data": {
      "components": [
        {"Reference":"U1","Quantity":1,"manufacturer":{"name":"Acme","mpn":"IC-1"}},
        {"Reference":"R1","Quantity":2,"manufacturer":{"name":"OhmCo","mpn":"R-0603"}}
      ]
    }
  }
}"#,
        )
        .unwrap();

        let artifacts = load_text_artifacts(std::slice::from_ref(&path)).unwrap();

        assert!(artifacts[0].text.contains("manufacturer.name"));
        assert!(artifacts[0].text.contains("manufacturer.mpn"));
        assert!(artifacts[0].text.contains("Acme"));
        assert!(artifacts[0].text.contains("IC-1"));
        assert!(artifacts[0].text.contains("R-0603"));
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

    fn write_minimal_xlsx(path: &std::path::Path, rows: &[&[&str]]) {
        use std::io::Write as _;

        let file = std::fs::File::create(path).unwrap();
        let mut zip = zip::ZipWriter::new(file);
        let options = zip::write::SimpleFileOptions::default();
        zip.start_file("[Content_Types].xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>
  <Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
</Types>"#,
        )
        .unwrap();
        zip.start_file("_rels/.rels", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>
</Relationships>"#,
        )
        .unwrap();
        zip.start_file("xl/workbook.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <sheets><sheet name="Sheet1" sheetId="1" r:id="rId1"/></sheets>
</workbook>"#,
        )
        .unwrap();
        zip.start_file("xl/_rels/workbook.xml.rels", options)
            .unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>
</Relationships>"#,
        )
        .unwrap();
        zip.start_file("xl/worksheets/sheet1.xml", options).unwrap();
        let mut sheet = String::from(
            r#"<?xml version="1.0" encoding="UTF-8"?>
<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetData>"#,
        );
        for (row_index, row) in rows.iter().enumerate() {
            sheet.push_str(&format!(r#"<row r="{}">"#, row_index + 1));
            for (column_index, cell) in row.iter().enumerate() {
                sheet.push_str(&format!(
                    r#"<c r="{}{}" t="inlineStr"><is><t>{}</t></is></c>"#,
                    xlsx_column_name(column_index),
                    row_index + 1,
                    xml_escape(cell)
                ));
            }
            sheet.push_str("</row>");
        }
        sheet.push_str("</sheetData></worksheet>");
        zip.write_all(sheet.as_bytes()).unwrap();
        zip.finish().unwrap();
    }

    fn write_two_sheet_xlsx(path: &std::path::Path, sheet1: &[&[&str]], sheet2: &[&[&str]]) {
        use std::io::Write as _;

        let file = std::fs::File::create(path).unwrap();
        let mut zip = zip::ZipWriter::new(file);
        let options = zip::write::SimpleFileOptions::default();
        zip.start_file("[Content_Types].xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>
  <Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
  <Override PartName="/xl/worksheets/sheet2.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
</Types>"#,
        )
        .unwrap();
        zip.start_file("_rels/.rels", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>
</Relationships>"#,
        )
        .unwrap();
        zip.start_file("xl/workbook.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <sheets>
    <sheet name="Top" sheetId="1" r:id="rId1"/>
    <sheet name="Bottom" sheetId="2" r:id="rId2"/>
  </sheets>
</workbook>"#,
        )
        .unwrap();
        zip.start_file("xl/_rels/workbook.xml.rels", options)
            .unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet2.xml"/>
</Relationships>"#,
        )
        .unwrap();
        zip.start_file("xl/worksheets/sheet1.xml", options).unwrap();
        zip.write_all(xlsx_sheet_xml(sheet1).as_bytes()).unwrap();
        zip.start_file("xl/worksheets/sheet2.xml", options).unwrap();
        zip.write_all(xlsx_sheet_xml(sheet2).as_bytes()).unwrap();
        zip.finish().unwrap();
    }

    fn xlsx_sheet_xml(rows: &[&[&str]]) -> String {
        let mut sheet = String::from(
            r#"<?xml version="1.0" encoding="UTF-8"?>
<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetData>"#,
        );
        for (row_index, row) in rows.iter().enumerate() {
            sheet.push_str(&format!(r#"<row r="{}">"#, row_index + 1));
            for (column_index, cell) in row.iter().enumerate() {
                sheet.push_str(&format!(
                    r#"<c r="{}{}" t="inlineStr"><is><t>{}</t></is></c>"#,
                    xlsx_column_name(column_index),
                    row_index + 1,
                    xml_escape(cell)
                ));
            }
            sheet.push_str("</row>");
        }
        sheet.push_str("</sheetData></worksheet>");
        sheet
    }

    fn write_formula_xlsx(path: &std::path::Path) {
        use std::io::Write as _;

        let file = std::fs::File::create(path).unwrap();
        let mut zip = zip::ZipWriter::new(file);
        let options = zip::write::SimpleFileOptions::default();
        zip.start_file("[Content_Types].xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>
  <Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
</Types>"#,
        )
        .unwrap();
        zip.start_file("_rels/.rels", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>
</Relationships>"#,
        )
        .unwrap();
        zip.start_file("xl/workbook.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <sheets><sheet name="Sheet1" sheetId="1" r:id="rId1"/></sheets>
</workbook>"#,
        )
        .unwrap();
        zip.start_file("xl/_rels/workbook.xml.rels", options)
            .unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>
</Relationships>"#,
        )
        .unwrap();
        zip.start_file("xl/worksheets/sheet1.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">
  <sheetData>
    <row r="1">
      <c r="A1" t="inlineStr"><is><t>Reference</t></is></c>
      <c r="B1" t="inlineStr"><is><t>Quantity</t></is></c>
      <c r="C1" t="inlineStr"><is><t>MPN</t></is></c>
    </row>
    <row r="2">
      <c r="A2" t="inlineStr"><is><t>R1</t></is></c>
      <c r="B2"><f>SUM(1,1)</f><v>2</v></c>
      <c r="C2" t="inlineStr"><is><t>RC0603</t></is></c>
    </row>
  </sheetData>
</worksheet>"#,
        )
        .unwrap();
        zip.finish().unwrap();
    }

    fn write_workbook_semantics_xlsx(path: &std::path::Path) {
        use std::io::Write as _;

        let file = std::fs::File::create(path).unwrap();
        let mut zip = zip::ZipWriter::new(file);
        let options = zip::write::SimpleFileOptions::default();
        zip.start_file("[Content_Types].xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>
  <Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>
  <Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
  <Override PartName="/xl/worksheets/sheet2.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
  <Override PartName="/xl/tables/table1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.table+xml"/>
  <Override PartName="/xl/drawings/drawing1.xml" ContentType="application/vnd.openxmlformats-officedocument.drawing+xml"/>
  <Override PartName="/xl/charts/chart1.xml" ContentType="application/vnd.openxmlformats-officedocument.drawingml.chart+xml"/>
  <Override PartName="/xl/pivotTables/pivotTable1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.pivotTable+xml"/>
  <Override PartName="/xl/comments1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.comments+xml"/>
</Types>"#,
        )
        .unwrap();
        zip.start_file("_rels/.rels", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>
</Relationships>"#,
        )
        .unwrap();
        zip.start_file("xl/workbook.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <sheets>
    <sheet name="ReleaseBOM" sheetId="1" r:id="rId1"/>
    <sheet name="SupplierData" sheetId="2" state="hidden" r:id="rId2"/>
  </sheets>
</workbook>"#,
        )
        .unwrap();
        zip.start_file("xl/styles.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">
  <numFmts count="1"><numFmt numFmtId="164" formatCode="0.000 &quot;pcs&quot;"/></numFmts>
  <fills count="3">
    <fill><patternFill patternType="none"/></fill>
    <fill><patternFill patternType="gray125"/></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FFFFEE00"/></patternFill></fill>
  </fills>
  <borders count="2"><border/><border><left style="thin"/></border></borders>
  <cellStyleXfs count="1"><xf numFmtId="0" fillId="0" borderId="0"/></cellStyleXfs>
  <cellXfs count="2">
    <xf numFmtId="0" fillId="0" borderId="0" xfId="0"/>
    <xf numFmtId="164" fillId="2" borderId="1" xfId="0" applyNumberFormat="1" applyFill="1" applyBorder="1"/>
  </cellXfs>
  <dxfs count="1"><dxf><fill><patternFill patternType="solid"><fgColor rgb="FFFF0000"/></patternFill></fill></dxf></dxfs>
</styleSheet>"#,
        )
        .unwrap();
        zip.start_file("xl/_rels/workbook.xml.rels", options)
            .unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet2.xml"/>
</Relationships>"#,
        )
        .unwrap();
        zip.start_file("xl/worksheets/sheet1.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <sheetData>
    <row r="1">
      <c r="A1" t="inlineStr"><is><t>Reference</t></is></c>
      <c r="B1" t="inlineStr"><is><t>Quantity</t></is></c>
      <c r="C1" t="inlineStr"><is><t>MPN</t></is></c>
    </row>
    <row r="2">
      <c r="A2" t="inlineStr"><is><t>R1</t></is></c>
      <c r="B2"><v>1</v></c>
      <c r="C2" t="inlineStr"><is><t>RC0603</t></is></c>
    </row>
  </sheetData>
  <autoFilter ref="A1:C2"/>
  <conditionalFormatting sqref="B2"><cfRule type="cellIs" priority="1" operator="greaterThan"><formula>0</formula></cfRule></conditionalFormatting>
  <dataValidations count="1"><dataValidation type="whole" allowBlank="1" sqref="B2"><formula1>0</formula1><formula2>100</formula2></dataValidation></dataValidations>
  <hyperlinks><hyperlink ref="C2" r:id="rId2"/></hyperlinks>
  <mergeCells count="1"><mergeCell ref="A1:B1"/></mergeCells>
  <tableParts count="1"><tablePart r:id="rId1"/></tableParts>
</worksheet>"#,
        )
        .unwrap();
        zip.start_file("xl/worksheets/_rels/sheet1.xml.rels", options)
            .unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/table" Target="../tables/table1.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/hyperlink" Target="https://example.invalid/part" TargetMode="External"/>
</Relationships>"#,
        )
        .unwrap();
        zip.start_file("xl/worksheets/sheet2.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">
  <sheetData>
    <row r="1">
      <c r="A1" t="inlineStr"><is><t>InternalNote</t></is></c>
    </row>
    <row r="2">
      <c r="A2" t="inlineStr"><is><t>Supplier-only alternates</t></is></c>
    </row>
  </sheetData>
</worksheet>"#,
        )
        .unwrap();
        zip.start_file("xl/tables/table1.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<table xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" id="1" name="ReleaseTable" displayName="ReleaseTable" ref="A1:C2" totalsRowShown="0">
  <autoFilter ref="A1:C2"/>
  <tableColumns count="3">
    <tableColumn id="1" name="Reference"/>
    <tableColumn id="2" name="Quantity"/>
    <tableColumn id="3" name="MPN"/>
  </tableColumns>
</table>"#,
        )
        .unwrap();
        zip.start_file("xl/drawings/drawing1.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<xdr:wsDr xmlns:xdr="http://schemas.openxmlformats.org/drawingml/2006/spreadsheetDrawing" xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main">
  <xdr:twoCellAnchor><xdr:clientData/></xdr:twoCellAnchor>
</xdr:wsDr>"#,
        )
        .unwrap();
        zip.start_file("xl/comments1.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<comments xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">
  <authors><author>reviewer</author></authors>
  <commentList><comment ref="C2" authorId="0"><text><r><t>approved alternate</t></r></text></comment></commentList>
</comments>"#,
        )
        .unwrap();
        zip.start_file("xl/charts/chart1.xml", options).unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<c:chartSpace xmlns:c="http://schemas.openxmlformats.org/drawingml/2006/chart">
  <c:chart><c:plotArea/></c:chart>
</c:chartSpace>"#,
        )
        .unwrap();
        zip.start_file("xl/pivotTables/pivotTable1.xml", options)
            .unwrap();
        zip.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<pivotTableDefinition xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" name="PivotTable1" cacheId="1">
</pivotTableDefinition>"#,
        )
        .unwrap();
        zip.start_file("xl/media/image1.png", options).unwrap();
        zip.write_all(b"not-a-real-png-for-metadata-only").unwrap();
        zip.finish().unwrap();
    }

    fn xlsx_column_name(mut index: usize) -> String {
        let mut name = String::new();
        loop {
            let remainder = index % 26;
            name.insert(0, (b'A' + remainder as u8) as char);
            index /= 26;
            if index == 0 {
                break;
            }
            index -= 1;
        }
        name
    }

    fn xml_escape(value: &str) -> String {
        value
            .replace('&', "&amp;")
            .replace('<', "&lt;")
            .replace('>', "&gt;")
            .replace('"', "&quot;")
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
            drc_report: None,
            input_hash: "hyperdrc-input-v1:test".to_string(),
            steps: vec![crate::conversion::ConversionStepLog {
                operation: "Gerber export".to_string(),
                command: "kicad-cli pcb export gerbers board.kicad_pcb".to_string(),
                status: "exit status: 0".to_string(),
                stdout: String::new(),
                stderr: String::new(),
            }],
            version: Some(crate::conversion::ConversionStepLog {
                operation: "version probe".to_string(),
                command: "kicad-cli --version".to_string(),
                status: "exit status: 0".to_string(),
                stdout: "kicad-cli 9.0".to_string(),
                stderr: String::new(),
            }),
            output_manifest: Vec::new(),
        };

        let layers = load_all_layers(
            std::slice::from_ref(&direct_path),
            std::slice::from_ref(&gerber_dir),
            &[],
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
        assert_eq!(
            layers[2].source.source_hash.as_deref(),
            Some("hyperdrc-input-v1:test")
        );
        assert_eq!(
            layers[2].source.transformation_history,
            vec![
                "kicad-cli --version".to_string(),
                "kicad-cli pcb export gerbers board.kicad_pcb".to_string()
            ]
        );

        let _ = fs::remove_dir_all(workspace);
    }

    #[test]
    fn load_all_layers_discovers_extracted_package_archives_recursively() {
        let workspace = PathBuf::from(format!(
            "/tmp/hyperdrc-archive-layers-{}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&workspace);
        fs::create_dir_all(&workspace).unwrap();
        let archive_path = workspace.join("release.zip");
        {
            let file = fs::File::create(&archive_path).unwrap();
            let mut writer = zip::ZipWriter::new(file);
            let options = zip::write::SimpleFileOptions::default();
            writer.start_file("nested/top.gtl", options).unwrap();
            use std::io::Write as _;
            writer.write_all(VALID_GERBER.as_bytes()).unwrap();
            writer.finish().unwrap();
        }
        let extracted =
            crate::package_archive::ExtractedPackages::extract(std::slice::from_ref(&archive_path))
                .unwrap();

        let layers = load_all_layers(&[], &[], extracted.packages(), &[]).unwrap();

        assert_eq!(layers.len(), 1);
        assert_eq!(layers[0].source.adapter, IoAdapter::PackageArchive);
        assert_eq!(
            layers[0].source.origin.as_deref(),
            Some(archive_path.to_str().unwrap())
        );
        let _ = fs::remove_dir_all(workspace);
    }

    #[test]
    fn load_layers_preserves_gerber_x2_file_metadata_for_manifest() {
        let process_id = process::id();
        let path = PathBuf::from(format!("/tmp/hyperdrc-x2-metadata-{process_id}.gbr"));
        fs::write(
            &path,
            "%TF.FileFunction,Copper,L1,Top*%\n%TF.FilePolarity,Positive*%\n%TF.SameCoordinates,PX1*%\n%TO.N,GND*%\n%TO.C,U1*%\n%TO.P,U1,1*%\n%TD.N*%\n%TD*%\n%LPD*%\n%SRX2Y1I1.0J0.0*%\nG04 trace*\n%MOMM*%\n%FSLAX46Y46*%\n%AMTHERM*1,1,0.5,0,0,0*%\n%ADD10C,1*%\nD10*\nG75*\nG36*\nG01X0Y0D02*\nG02X4000000Y0I2000000J0D01*\nG37*\n%SR*%\nM02*\n",
        )
        .unwrap();

        let layers = load_layers(std::slice::from_ref(&path)).unwrap();
        assert_eq!(
            layers[0].gerber_metadata.file_function.as_deref(),
            Some("Copper,L1,Top")
        );
        assert_eq!(
            layers[0].gerber_metadata.file_polarity.as_deref(),
            Some("Positive")
        );
        assert_eq!(
            layers[0].gerber_metadata.same_coordinates.as_deref(),
            Some("PX1")
        );
        assert_eq!(
            layers[0].gerber_image_setup.units,
            Some(crate::gerber_metadata::GerberUnits::Millimeters)
        );
        assert_eq!(
            layers[0].gerber_image_setup.coordinate_format,
            Some(crate::gerber_metadata::GerberCoordinateFormat {
                integer_digits: 4,
                decimal_digits: 6
            })
        );
        assert_eq!(
            layers[0].source.source_units.as_deref(),
            Some("millimeters")
        );
        assert_eq!(
            layers[0].source.normalized_units.as_deref(),
            Some("millimeters")
        );
        assert_eq!(layers[0].gerber_aperture_definitions.len(), 1);
        assert_eq!(layers[0].gerber_aperture_macros.len(), 1);
        assert_eq!(layers[0].gerber_aperture_definitions[0].d_code, 10);
        assert_eq!(layers[0].gerber_aperture_uses.len(), 2);
        assert_eq!(layers[0].gerber_coordinate_operations.len(), 2);
        assert_eq!(layers[0].gerber_polarity_changes.len(), 1);
        assert_eq!(layers[0].gerber_region_events.len(), 2);
        assert_eq!(layers[0].gerber_step_repeat_events.len(), 2);
        assert_eq!(layers[0].gerber_interpolation_events.len(), 2);
        assert_eq!(layers[0].gerber_quadrant_events.len(), 1);
        assert_eq!(layers[0].gerber_object_metadata.len(), 3);
        assert_eq!(layers[0].gerber_attribute_deletes.len(), 2);

        let cli = Cli::parse_from(["hyperdrc", path.to_str().unwrap()]);
        let package_inputs = package_inputs(&cli, Default::default());
        let rules = default_rules();
        let manifest = manifest_input(&cli, &rules, &layers, &[], &package_inputs);

        assert_eq!(
            manifest.gerber_layers[0].file_function.as_deref(),
            Some("Copper,L1,Top")
        );
        assert_eq!(
            manifest.gerber_layers[0].file_polarity.as_deref(),
            Some("Positive")
        );
        assert_eq!(
            manifest.gerber_layers[0].same_coordinates.as_deref(),
            Some("PX1")
        );
        assert_eq!(
            manifest.gerber_layers[0].units.as_deref(),
            Some("millimeters")
        );
        assert_eq!(
            manifest.gerber_layers[0].coordinate_format.as_deref(),
            Some("4:6")
        );

        let _ = fs::remove_file(path);
    }

    #[test]
    fn load_all_layers_wraps_parse_errors_with_file_path_context() {
        let process_id = process::id();
        let workspace = PathBuf::from(format!("/tmp/hyperdrc-load-all-layers-parse-{process_id}"));
        let _ = fs::remove_dir_all(&workspace);
        fs::create_dir_all(&workspace).unwrap();
        let invalid_file = workspace.join("invalid.gbr");
        fs::write(&invalid_file, "this is not a gerber file\n").unwrap();

        let error =
            load_all_layers(std::slice::from_ref(&invalid_file), &[], &[], &[]).unwrap_err();

        let message = format!("{error}");
        assert!(message.contains("failed to parse Gerber"));
        assert!(message.contains("invalid.gbr"));

        let _ = fs::remove_dir_all(workspace);
    }
}
