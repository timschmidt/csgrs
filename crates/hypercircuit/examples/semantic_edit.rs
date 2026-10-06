//! Commit, persist, undo, and redo one exact route edit on an imported KiCad board.

use std::error::Error;
use std::path::Path;

use hypercircuit::{
    BoardId, CircuitId, DesignEdit, DesignEditBatch, DesignEditId, DesignHistory, DesignRevision,
    KiCadExportOptions, KiCadImportOptions, KiCadImportReport, Real, RouteId, SemanticDocument,
};

fn main() -> Result<(), Box<dyn Error>> {
    let mut arguments = std::env::args().skip(1);
    let input = arguments
        .next()
        .ok_or("usage: semantic_edit INPUT OUTPUT ROUTE WIDTH_NUMERATOR WIDTH_DENOMINATOR")?;
    let output = arguments
        .next()
        .ok_or("usage: semantic_edit INPUT OUTPUT ROUTE WIDTH_NUMERATOR WIDTH_DENOMINATOR")?;
    let route = arguments
        .next()
        .ok_or("usage: semantic_edit INPUT OUTPUT ROUTE WIDTH_NUMERATOR WIDTH_DENOMINATOR")?;
    let numerator = arguments
        .next()
        .ok_or("usage: semantic_edit INPUT OUTPUT ROUTE WIDTH_NUMERATOR WIDTH_DENOMINATOR")?
        .parse::<i64>()?;
    let denominator = arguments
        .next()
        .ok_or("usage: semantic_edit INPUT OUTPUT ROUTE WIDTH_NUMERATOR WIDTH_DENOMINATOR")?
        .parse::<i64>()?;
    let width = (Real::from(numerator) / Real::from(denominator))?;
    let copper_thickness = (Real::from(35) / Real::from(1_000))?;
    let circuit_id = CircuitId::new("semantic-edit")?;
    let board_id = BoardId::new("semantic-edit")?;
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
    let document = SemanticDocument::new(imported.circuit, None)?.with_pcb(imported.layout)?;
    let mut history = DesignHistory::new(document)?;
    let committed = history.commit(DesignEditBatch {
        id: DesignEditId::new("cli-route-width")?,
        expected_revision: DesignRevision::default(),
        editor: "semantic-edit-example".into(),
        edits: vec![DesignEdit::SetRouteWidth {
            route: RouteId::new(route)?,
            width,
        }],
    })?;
    let undone = history.undo()?;
    let retained_json = history.to_json_pretty()?;
    let mut history = DesignHistory::from_json(&retained_json)?;
    let redone = history.redo()?;
    let document = history.document();
    let exported = document
        .pcb
        .as_ref()
        .expect("imported document has a PCB")
        .export_kicad(&document.circuit, KiCadExportOptions::default())?;
    let output_path = Path::new(&output);
    std::fs::write(output_path, exported.board)?;
    if let Some(rules) = exported.design_rules {
        std::fs::write(output_path.with_extension("kicad_dru"), rules)?;
    }
    if let Some(project) = exported.project {
        std::fs::write(output_path.with_extension("kicad_pro"), project)?;
    }
    println!(
        "committed {} at revision {}, undo={}, persisted bytes={}, redo={}; export omissions={}",
        committed.original_batch.as_str(),
        committed.replay.to_revision.value(),
        undone.replay.to_revision.value(),
        retained_json.len(),
        redone.replay.to_revision.value(),
        exported.omissions.len()
    );
    Ok(())
}
