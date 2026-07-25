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
    let mut completed = 0;
    let mut blocked = 0;
    for (slug, source) in BOARDS {
        let document = SemanticDocument::from_json(source)
            .unwrap_or_else(|error| panic!("decode Easyduino {slug}: {error}"));
        println!("easyduino-full-pipeline/{slug}: starting");
        let started = Instant::now();
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
                    "easyduino-full-pipeline/{slug}: typed termination after {:?}: {error}",
                    started.elapsed(),
                );
            }
        }
    }
    println!(
        "easyduino-full-pipeline/all: {:?}; {completed} completed; {blocked} typed terminations",
        suite_started.elapsed()
    );
}
