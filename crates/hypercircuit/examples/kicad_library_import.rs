//! Convert standalone KiCad symbol and footprint entries into a portable part artifact.

use std::io::{Error, ErrorKind};
use std::path::PathBuf;

use hypercircuit::{
    CircuitPackageName, DeviceModelId, DeviceModelKind, KiCadPartLibraryImportOptions,
    KiCadPartLibraryImportReport, LandPatternId, PartLibraryArtifact, SchematicSymbolDefinitionId,
};
use semver::Version;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut arguments = std::env::args_os().skip(1);
    let symbol_library = arguments.next().map(PathBuf::from).ok_or_else(usage)?;
    let symbol_name = arguments
        .next()
        .and_then(|value| value.into_string().ok())
        .ok_or_else(usage)?;
    let footprint = arguments.next().map(PathBuf::from).ok_or_else(usage)?;
    let export_name = arguments
        .next()
        .and_then(|value| value.into_string().ok())
        .ok_or_else(usage)?;
    let output = arguments.next().map(PathBuf::from).ok_or_else(usage)?;
    if arguments.next().is_some() {
        return Err(usage().into());
    }

    let report = KiCadPartLibraryImportReport::from_paths(
        symbol_library,
        &symbol_name,
        footprint,
        KiCadPartLibraryImportOptions::new(
            &export_name,
            DeviceModelId::new(format!("{export_name}.model"))?,
            DeviceModelKind::Custom(format!("KiCad symbol {symbol_name}")),
            SchematicSymbolDefinitionId::new(format!("{export_name}.symbol"))?,
            LandPatternId::new(format!("{export_name}.footprint"))?,
        ),
    )?;
    let artifact = PartLibraryArtifact::new(
        CircuitPackageName::new(&export_name)?,
        Version::new(0, 1, 0),
        vec![report.part],
    )?;
    std::fs::write(&output, artifact.to_json_pretty()?)?;
    println!(
        "wrote {} with {} exact numeric imports and {} explicit omission(s)",
        output.display(),
        report.numeric_imports.len(),
        report.omissions.len()
    );
    for omission in report.omissions {
        println!("  {omission:?}");
    }
    Ok(())
}

fn usage() -> Error {
    Error::new(
        ErrorKind::InvalidInput,
        "usage: kicad_library_import SYMBOLS.kicad_sym SYMBOL FOOTPRINT.kicad_mod EXPORT OUTPUT.json",
    )
}
