//! Slow, full-fidelity Easyduino release-pipeline benchmark.
//!
//! Unlike the routine regression tests, this retains authored zones, exact
//! zone pouring, aggregate copper/process images, fabrication, CAM audit, and
//! assembly audit. Run it explicitly:
//!
//! `cargo bench --bench easyduino_full_pipeline --features "drc interchange"`

use std::time::Instant;

use hypercircuit::{ReleasePreparationOptions, SemanticDocument};

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
    let materialize_only =
        std::env::var("EASYDUINO_STAGE").is_ok_and(|stage| stage == "materialize");
    let mut completed = 0;
    let mut blocked = 0;
    for (slug, source) in BOARDS.iter().copied().filter(|(slug, _)| {
        requested_board
            .as_deref()
            .is_none_or(|requested| requested == *slug)
    }) {
        let document = SemanticDocument::from_json(source)
            .unwrap_or_else(|error| panic!("decode Easyduino {slug}: {error}"));
        println!("easyduino-full-pipeline/{slug}: starting");
        let started = Instant::now();
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
            match layout.materialize(
                &document.circuit,
                ReleasePreparationOptions::default().materialization,
            ) {
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
        match document.prepare_release(ReleasePreparationOptions::default()) {
            Ok(report) => {
                completed += 1;
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
}
