//! Import and re-export the supported editable KiCad PCB subset.

use std::error::Error;
use std::path::Path;

use hypercircuit::{
    BoardId, CircuitId, KiCadExportOptions, KiCadImportOptions, KiCadImportReport, Real,
};

fn main() -> Result<(), Box<dyn Error>> {
    let mut arguments = std::env::args().skip(1);
    let input = arguments
        .next()
        .ok_or("usage: kicad_roundtrip INPUT.kicad_pcb OUTPUT.kicad_pcb")?;
    let output = arguments
        .next()
        .ok_or("usage: kicad_roundtrip INPUT.kicad_pcb OUTPUT.kicad_pcb")?;
    let copper_thickness = (Real::from(35) / Real::from(1_000))?;
    let circuit_id = CircuitId::new("imported-kicad")?;
    let board_id = BoardId::new("imported-kicad")?;
    let import_options = || {
        KiCadImportOptions::new(
            circuit_id.clone(),
            board_id.clone(),
            copper_thickness.clone(),
        )
    };
    let input_path = Path::new(&input);
    let input_rules = input_path.with_extension("kicad_dru");
    let input_project = input_path.with_extension("kicad_pro");
    let imported = if input_project.is_file() {
        KiCadImportReport::from_project_paths(
            input_path,
            &input_project,
            input_rules.is_file().then_some(input_rules.as_path()),
            import_options(),
        )
    } else if input_rules.is_file() {
        KiCadImportReport::from_paths(input_path, &input_rules, import_options())
    } else {
        KiCadImportReport::from_path(input_path, import_options())
    }?;
    let exported = imported
        .layout
        .export_kicad(&imported.circuit, KiCadExportOptions::default())?;
    let output_path = Path::new(&output);
    std::fs::write(output_path, exported.board)?;
    let emitted_rules = exported.design_rules.is_some();
    if let Some(rules) = exported.design_rules {
        std::fs::write(output_path.with_extension("kicad_dru"), rules)?;
    }
    let emitted_project = exported.project.is_some();
    if let Some(project) = exported.project {
        std::fs::write(output_path.with_extension("kicad_pro"), project)?;
    }
    println!(
        "imported {} nets, {} placements, {} routes; project={emitted_project}, rules={emitted_rules}, import omissions={}, export omissions={}",
        imported.circuit.nets.len(),
        imported.layout.placements.len(),
        imported.layout.routes.len(),
        imported.omissions.len(),
        exported.omissions.len()
    );
    Ok(())
}
