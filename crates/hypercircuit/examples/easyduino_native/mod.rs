use std::error::Error;
use std::time::Instant;

use hypercircuit::{DrcReadinessPolicy, HyperDrcHandoff, MaterializationOptions, SemanticDocument};

pub fn run(slug: &str, source: &str) -> Result<(), Box<dyn Error>> {
    let document = SemanticDocument::from_json(source)?;
    let mut layout = document
        .pcb
        .as_ref()
        .expect("Easyduino native fixture retains a PCB")
        .clone();
    let authored_zones = layout.zones.len();

    // Routine examples exercise all independently-authored copper. KiCad's
    // cached filled-zone result is deliberately not imported; exact re-pouring
    // and CAM aggregation remain available in the full benchmark.
    layout.zones.clear();
    let started = Instant::now();
    let materialized = layout.materialize(
        &document.circuit,
        MaterializationOptions {
            circular_segments: 8,
            aggregate_layer_images: false,
            ..MaterializationOptions::default()
        },
    )?;
    let handoff = HyperDrcHandoff::from_materialization(&layout, &materialized);
    let readiness = handoff.run_readiness(&DrcReadinessPolicy::default());

    println!(
        "Easyduino {slug}: {} nets, {} placements, {} routes, {} vias, \
         {authored_zones} authored zones, {} materialized copper features, \
         {} HyperDRC findings, {} handoff omissions ({:?})",
        document.circuit.nets.len(),
        layout.placements.len(),
        layout.routes.len(),
        layout.vias.len(),
        materialized.copper_features.len(),
        readiness.violations.len(),
        handoff.omissions.len(),
        started.elapsed(),
    );
    Ok(())
}
