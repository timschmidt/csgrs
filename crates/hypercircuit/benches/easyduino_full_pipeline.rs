//! Slow, full-fidelity Easyduino release-pipeline benchmark.
//!
//! Unlike the routine regression tests, this retains authored zones, exact
//! zone pouring, aggregate copper/process images, fabrication, CAM audit, and
//! assembly audit. Run it explicitly:
//!
//! `cargo bench --bench easyduino_full_pipeline --features "drc interchange"`

use std::time::Instant;
use std::{collections::BTreeMap, fs, path::Path};

use hypercircuit::{
    AssemblyOutputs, FabricationPackage, HyperDrcHandoff, ReleaseOptions, SemanticDocument,
};
use hyperdrc::PcbSketchExt;

const BOARDS: &[(&str, &str)] = &[
    (
        "nano",
        include_str!("../tests/fixtures/easyduino/native/nano.hypercircuit.json"),
    ),
    (
        "uno",
        include_str!("../tests/fixtures/easyduino/native/uno.hypercircuit.json"),
    ),
    (
        "esp32",
        include_str!("../tests/fixtures/easyduino/native/esp32.hypercircuit.json"),
    ),
    (
        "esp32s3",
        include_str!("../tests/fixtures/easyduino/native/esp32s3.hypercircuit.json"),
    ),
    (
        "rp2040",
        include_str!("../tests/fixtures/easyduino/native/rp2040.hypercircuit.json"),
    ),
    (
        "stm32f103",
        include_str!("../tests/fixtures/easyduino/native/stm32f103.hypercircuit.json"),
    ),
];

fn main() {
    let suite_started = Instant::now();
    let requested_board = std::env::var("EASYDUINO_BOARD").ok();
    let requested_check = std::env::var("EASYDUINO_CHECK").ok();
    let materialize_only =
        std::env::var("EASYDUINO_STAGE").is_ok_and(|stage| stage == "materialize");
    let profile_stages = std::env::var("EASYDUINO_STAGE").is_ok_and(|stage| stage == "profile");
    let profile_cam = std::env::var("EASYDUINO_STAGE").is_ok_and(|stage| stage == "cam");
    let mut completed = 0;
    let mut blocked = 0;
    let mut snapshots = Vec::new();
    for (slug, source) in BOARDS.iter().copied().filter(|(slug, _)| {
        requested_board
            .as_deref()
            .is_none_or(|requested| requested == *slug)
    }) {
        let document = SemanticDocument::from_json(source)
            .unwrap_or_else(|error| panic!("decode Easyduino {slug}: {error}"));
        println!("easyduino-full-pipeline/{slug}: starting");
        let started = Instant::now();
        if profile_stages || profile_cam {
            let options = ReleaseOptions::default();
            let mut layout = document
                .pcb
                .clone()
                .unwrap_or_else(|| panic!("Easyduino {slug} has no PCB"));
            let stage = Instant::now();
            let placement = layout.resolve_placement_constraints(&document.circuit);
            assert!(placement.is_satisfied());
            layout.placements = placement.placements;
            println!(
                "easyduino-full-pipeline/{slug}: placement {:?}",
                stage.elapsed()
            );
            let stage = Instant::now();
            let materialized = layout
                .materialize(&document.circuit, options.materialization)
                .unwrap();
            println!(
                "easyduino-full-pipeline/{slug}: materialization {:?}",
                stage.elapsed()
            );
            if profile_stages {
                let stage = Instant::now();
                let handoff = HyperDrcHandoff::from_materialization_with_context(
                    &layout,
                    &materialized,
                    &document.circuit,
                    &options.pcb_materials,
                    Some(&document.design_intent),
                    Some(&document.test_intent),
                );
                for copper in &handoff.copper_layers {
                    let polygons = copper.sketch.to_multipolygon().0;
                    println!(
                        "easyduino-full-pipeline/{slug}: copper {} has {} polygons / {} rings / {} vertices",
                        copper.name,
                        polygons.len(),
                        polygons
                            .iter()
                            .map(|polygon| 1 + polygon.interiors().len())
                            .sum::<usize>(),
                        polygons
                            .iter()
                            .map(|polygon| {
                                polygon.exterior().0.len()
                                    + polygon
                                        .interiors()
                                        .iter()
                                        .map(|ring| ring.0.len())
                                        .sum::<usize>()
                            })
                            .sum::<usize>(),
                    );
                }
                for process in &handoff.process_layers {
                    let polygons = process.sketch.to_multipolygon().0;
                    println!(
                        "easyduino-full-pipeline/{slug}: process {} has {} polygons / {} rings / {} vertices",
                        process.name,
                        polygons.len(),
                        polygons
                            .iter()
                            .map(|polygon| 1 + polygon.interiors().len())
                            .sum::<usize>(),
                        polygons
                            .iter()
                            .map(|polygon| {
                                polygon.exterior().0.len()
                                    + polygon
                                        .interiors()
                                        .iter()
                                        .map(|ring| ring.0.len())
                                        .sum::<usize>()
                            })
                            .sum::<usize>(),
                    );
                }
                let mut drc_findings = 0;
                for check in hyperdrc::default_checks().iter().filter(|check| {
                    requested_check
                        .as_deref()
                        .is_none_or(|requested| requested == check.slug())
                }) {
                    println!(
                        "easyduino-full-pipeline/{slug}: HyperDRC {} starting",
                        check.slug()
                    );
                    let check_started = Instant::now();
                    let report = handoff.run_readiness_selected(&options.drc, [*check]);
                    drc_findings += report.violations.len();
                    println!(
                        "easyduino-full-pipeline/{slug}: HyperDRC {} {:?} ({} findings)",
                        check.slug(),
                        check_started.elapsed(),
                        report.violations.len()
                    );
                }
                println!(
                    "easyduino-full-pipeline/{slug}: HyperDRC total {:?} ({drc_findings} findings)",
                    stage.elapsed(),
                );
            }
            let stage = Instant::now();
            let fabrication = FabricationPackage::from_materialization_with_options(
                &layout,
                &materialized,
                options.fabrication,
            )
            .unwrap();
            println!(
                "easyduino-full-pipeline/{slug}: fabrication {:?}",
                stage.elapsed()
            );
            let stage = Instant::now();
            let cam = fabrication.audit_cam_round_trip();
            println!(
                "easyduino-full-pipeline/{slug}: CAM audit {:?} ({} issues)",
                stage.elapsed(),
                cam.issues.len()
            );
            let stage = Instant::now();
            let assembly = AssemblyOutputs::from_design(&document.circuit, &layout).unwrap();
            let assembly_audit = assembly.audit_csv_round_trip();
            println!(
                "easyduino-full-pipeline/{slug}: assembly {:?} ({} issues)",
                stage.elapsed(),
                assembly_audit.issues.len()
            );
            completed += 1;
            continue;
        }
        if materialize_only {
            let mut layout = document
                .pcb
                .clone()
                .unwrap_or_else(|| panic!("Easyduino {slug} has no PCB"));
            let placement = layout.resolve_placement_constraints(&document.circuit);
            assert!(
                placement.is_satisfied(),
                "Easyduino {slug} placement is not satisfied"
            );
            layout.placements.clone_from(&placement.placements);
            match layout.materialize(&document.circuit, ReleaseOptions::default().materialization) {
                Ok(report) => {
                    let mut aggregate_blockers = 0;
                    for image in &report.copper_layers {
                        if let Some(blocker) = &image.blocker {
                            aggregate_blockers += 1;
                            println!(
                                "easyduino-full-pipeline/{slug}: copper layer {} blocker: {blocker}",
                                image.layer.0
                            );
                        }
                    }
                    for image in &report.process_layers {
                        if let Some(blocker) = &image.blocker {
                            aggregate_blockers += 1;
                            println!(
                                "easyduino-full-pipeline/{slug}: process layer {:?} blocker: {blocker}",
                                image.role
                            );
                        }
                    }
                    if aggregate_blockers == 0 {
                        completed += 1;
                    } else {
                        blocked += 1;
                    }
                    println!(
                        "easyduino-full-pipeline/{slug}: materialized in {:?}",
                        started.elapsed()
                    );
                }
                Err(error) => {
                    blocked += 1;
                    println!(
                        "easyduino-full-pipeline/{slug}: materialization terminated after {:?}: {error:?}",
                        started.elapsed(),
                    );
                }
            }
            continue;
        }
        match document.release_report(ReleaseOptions::default()) {
            Ok(report) => {
                completed += 1;
                let mut finding_counts = BTreeMap::<String, usize>::new();
                for violation in &report.drc.violations {
                    *finding_counts.entry(violation.check.clone()).or_default() += 1;
                }
                snapshots.push(serde_json::json!({
                    "board": slug,
                    "capability_profile": {
                        "id": &report.drc.capability_profile_id,
                        "revision": &report.drc.capability_profile_revision,
                        "digest": &report.drc.capability_profile_digest,
                    },
                    "coverage": &report.drc.coverage,
                    "finding_counts": finding_counts,
                    "release_blockers": report.release_blockers().iter().map(|value| format!("{value:?}")).collect::<Vec<_>>(),
                    "cam_round_trip_clean": report.cam_round_trip.is_release_clean(),
                    "assembly_round_trip_clean": report.assembly_round_trip.is_release_clean(),
                }));
                println!(
                    "easyduino-full-pipeline/{slug}: completed in {:?}; \
                     {} DRC findings; {} release blockers",
                    started.elapsed(),
                    report.drc.violations.len(),
                    report.release_blockers().len(),
                );
            }
            Err(error) => {
                blocked += 1;
                println!(
                    "easyduino-full-pipeline/{slug}: typed termination after {:?}: {error}; {error:?}",
                    started.elapsed(),
                );
            }
        }
    }
    println!(
        "easyduino-full-pipeline/all: {:?}; {completed} completed; {blocked} typed terminations",
        suite_started.elapsed()
    );
    assert!(
        completed > 0,
        "no Easyduino board matched the benchmark filter"
    );
    assert_eq!(
        blocked, 0,
        "Easyduino full-fidelity pipeline retained blockers"
    );
    if requested_board.is_none() && !materialize_only && !profile_stages && !profile_cam {
        let mut bytes = serde_json::to_vec_pretty(&snapshots).unwrap();
        bytes.push(b'\n');
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/easyduino/ordinary-release.json");
        if std::env::var_os("UPDATE_HYPERCIRCUIT_EASYDUINO_RELEASE_SNAPSHOT").is_some() {
            fs::write(&path, &bytes).unwrap();
        } else {
            assert_eq!(
                bytes,
                fs::read(&path).unwrap_or_else(|error| panic!(
                    "read {}: {error}; regenerate with UPDATE_HYPERCIRCUIT_EASYDUINO_RELEASE_SNAPSHOT=1",
                    path.display()
                )),
                "ordinary Easyduino release snapshot changed"
            );
        }
    }
}
