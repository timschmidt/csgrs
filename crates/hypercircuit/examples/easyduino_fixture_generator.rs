//! Regenerate native HyperCircuit fixtures from the pinned Easyduino sources.
//!
//! This maintenance tool is intentionally separate from the native examples:
//! those examples consume only HyperCircuit semantic documents and never invoke
//! the KiCad importer at runtime.

use std::error::Error;
use std::path::Path;
use std::time::Instant;

use hypercircuit::{
    BoardId, CircuitId, DrcReadinessPolicy, HyperDrcHandoff, KiCadImportOptions, KiCadImportReport,
    MaterializationOptions, Real, SemanticDocument,
};

const BOARDS: &[&str] = &["nano", "uno", "esp32", "esp32s3", "rp2040", "stm32f103"];

fn main() -> Result<(), Box<dyn Error>> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/easyduino");
    let copper_thickness = (Real::from(35) / Real::from(1_000))?;

    for slug in BOARDS {
        let source = root.join("upstream").join(slug);
        let imported = KiCadImportReport::from_project_paths(
            &source.join("board.kicad_pcb"),
            &source.join("board.kicad_pro"),
            None,
            KiCadImportOptions::new(
                CircuitId::new(format!("easyduino-{slug}"))?,
                BoardId::new(format!("easyduino-{slug}"))?,
                copper_thickness.clone(),
            ),
        )?;
        let document = SemanticDocument::new(imported.circuit, None)?.with_pcb(imported.layout)?;
        let mut layout = document
            .pcb
            .as_ref()
            .expect("native fixture has a PCB")
            .clone();
        let retained_zone_count = layout.zones.len();
        // KiCad's filled-zone cache is intentionally not part of the editable
        // semantic import. Keep the authored zones in the native fixture, while
        // this regeneration-time smoke check exercises the independently
        // authored pads, routes, and vias without re-pouring those zones.
        layout.zones.clear();
        println!("checking native Easyduino fixture: {slug}");
        let started = Instant::now();
        let materialization = layout.materialize(
            &document.circuit,
            &hypercircuit::MaterializationContext::STRICT,
            MaterializationOptions {
                circular_segments: 8,
                aggregate_layer_images: false,
                ..MaterializationOptions::default()
            },
        )?;
        println!("  materialization: {:?}", started.elapsed());
        let started = Instant::now();
        let handoff = HyperDrcHandoff::from_materialization(&layout, &materialization);
        println!("  HyperDRC handoff: {:?}", started.elapsed());
        let started = Instant::now();
        let drc = handoff.run_readiness(&DrcReadinessPolicy::default());
        println!("  HyperDRC readiness: {:?}", started.elapsed());
        std::fs::write(
            root.join("native")
                .join(format!("{slug}.hypercircuit.json")),
            document.to_json_pretty()?,
        )?;
        println!(
            "generated native Easyduino fixture: {slug} ({} HyperDRC violations, {} handoff omissions)",
            drc.violations.len(),
            handoff.omissions.len(),
        );
        println!("  retained authored zones: {retained_zone_count}");
    }
    Ok(())
}
