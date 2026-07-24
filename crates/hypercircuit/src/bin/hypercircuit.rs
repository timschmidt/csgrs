//! Project-oriented command-line workflows over retained HyperCircuit documents.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

use hypercircuit::{
    AssemblyOutputs, KiCadExportOptions, PcbSvgOptions, ProjectDesignProvider, ProjectManifest,
    ProjectProviderKind, ReleasePreparationOptions, SemanticDocument,
};

const PROJECT_MANIFEST_NAME: &str = "hypercircuit.toml";

struct LoadedDesign {
    document: SemanticDocument,
    release_options: ReleasePreparationOptions,
}

const USAGE: &str = "\
hypercircuit — validate and release retained circuit projects

USAGE:
    hypercircuit check [design.json | design-name]
    hypercircuit ir [design-name] [--out design.json]
    hypercircuit snapshot [design-name] [--out snapshot.json]
    hypercircuit bom [design-name] [--out bom.csv]
    hypercircuit export-kicad <design.json | design-name> <board.kicad_pcb>
    hypercircuit export-svg <design.json | design-name> <board.svg>
    hypercircuit release <design.json | design-name> <output-directory>
    hypercircuit --help
    hypercircuit --version

When a design argument is omitted or is not an existing file, HyperCircuit
searches the current directory and its ancestors for `hypercircuit.toml`, then
executes the selected command or Cargo provider. An omitted name selects the
manifest's `default-design`. Source-attributed `[[pcb-materials]]` declarations
in that manifest are applied to controlled-impedance `check` and `release`.
Direct JSON paths receive no implicit material catalog.

`check` validates semantic JSON and, for PCB documents, runs the complete
electrical, placement, geometry, HyperDRC, CAM, and assembly release gates.
`release` writes audited fabrication and assembly files even when reviewable
gate findings remain, and exits with status 2 in that case.
";

fn main() -> ExitCode {
    match run(env::args().skip(1).collect()) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(2),
        Err(error) => {
            eprintln!("hypercircuit: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(arguments: Vec<String>) -> Result<bool, String> {
    let Some(command) = arguments.first().map(String::as_str) else {
        print!("{USAGE}");
        return Ok(false);
    };
    match command {
        "-h" | "--help" | "help" => {
            print!("{USAGE}");
            Ok(true)
        }
        "-V" | "--version" | "version" => {
            println!("hypercircuit {}", env!("CARGO_PKG_VERSION"));
            Ok(true)
        }
        "check" => {
            expect_arity_range(&arguments, 1, 2)?;
            check(&load_design(arguments.get(1).map(String::as_str))?)
        }
        "ir" | "snapshot" => {
            let (design, output) = design_and_output(&arguments[1..])?;
            emit_semantic_document(&load_design(design)?.document, output.as_deref())
        }
        "bom" => {
            let (design, output) = design_and_output(&arguments[1..])?;
            emit_bom(&load_design(design)?.document, output.as_deref())
        }
        "export-kicad" => {
            expect_arity(&arguments, 3)?;
            export_kicad(
                &load_design(Some(&arguments[1]))?.document,
                Path::new(&arguments[2]),
            )
        }
        "export-svg" => {
            expect_arity(&arguments, 3)?;
            export_svg(
                &load_design(Some(&arguments[1]))?.document,
                Path::new(&arguments[2]),
            )
        }
        "release" => {
            expect_arity(&arguments, 3)?;
            release(&load_design(Some(&arguments[1]))?, Path::new(&arguments[2]))
        }
        other => Err(format!("unknown command `{other}`\n\n{USAGE}")),
    }
}

fn expect_arity_range(arguments: &[String], minimum: usize, maximum: usize) -> Result<(), String> {
    if (minimum..=maximum).contains(&arguments.len()) {
        Ok(())
    } else {
        Err(format!("invalid arguments\n\n{USAGE}"))
    }
}

fn expect_arity(arguments: &[String], expected: usize) -> Result<(), String> {
    expect_arity_range(arguments, expected, expected)
}

fn design_and_output(arguments: &[String]) -> Result<(Option<&str>, Option<PathBuf>), String> {
    let mut design = None;
    let mut output = None;
    let mut index = 0;
    if arguments
        .first()
        .is_some_and(|argument| argument != "--out")
    {
        design = Some(arguments[0].as_str());
        index = 1;
    }
    if index < arguments.len() {
        if arguments.get(index).map(String::as_str) != Some("--out") {
            return Err(format!("invalid arguments\n\n{USAGE}"));
        }
        let path = arguments
            .get(index + 1)
            .ok_or_else(|| format!("`--out` requires a path\n\n{USAGE}"))?;
        output = Some(PathBuf::from(path));
        index += 2;
    }
    if index != arguments.len() {
        return Err(format!("invalid arguments\n\n{USAGE}"));
    }
    Ok((design, output))
}

fn load_design(selector: Option<&str>) -> Result<LoadedDesign, String> {
    if let Some(selector) = selector {
        let path = Path::new(selector);
        if path.is_file() {
            return load_document(path);
        }
    }
    load_project_document(selector)
}

fn load_document(path: &Path) -> Result<LoadedDesign, String> {
    let json = fs::read_to_string(path)
        .map_err(|error| format!("cannot read {}: {error}", path.display()))?;
    let document = SemanticDocument::from_json(&json)
        .map_err(|error| format!("cannot load {}: {error}", path.display()))?;
    Ok(LoadedDesign {
        document,
        release_options: ReleasePreparationOptions::default(),
    })
}

fn load_project_document(design: Option<&str>) -> Result<LoadedDesign, String> {
    let manifest_path = find_project_manifest()?;
    let source = fs::read_to_string(&manifest_path)
        .map_err(|error| format!("cannot read {}: {error}", manifest_path.display()))?;
    let manifest = ProjectManifest::from_toml(&source)
        .map_err(|error| format!("cannot load {}: {error}", manifest_path.display()))?;
    let (name, provider) = manifest
        .design(design)
        .map_err(|error| format!("cannot select project design: {error}"))?;
    let directory = manifest_path
        .parent()
        .expect("a discovered manifest always has a parent");
    let output = execute_provider(provider, directory)
        .map_err(|error| format!("provider for design `{name}` failed: {error}"))?;
    let json = String::from_utf8(output).map_err(|error| {
        format!("provider for design `{name}` emitted non-UTF-8 output: {error}")
    })?;
    let document = SemanticDocument::from_json(&json).map_err(|error| {
        format!("provider for design `{name}` emitted invalid semantic JSON: {error}")
    })?;
    let pcb_materials = manifest
        .pcb_material_library()
        .map_err(|error| format!("cannot resolve project PCB materials: {error}"))?;
    Ok(LoadedDesign {
        document,
        release_options: ReleasePreparationOptions {
            pcb_materials,
            ..ReleasePreparationOptions::default()
        },
    })
}

fn find_project_manifest() -> Result<PathBuf, String> {
    let mut directory = env::current_dir()
        .map_err(|error| format!("cannot determine current directory: {error}"))?;
    loop {
        let candidate = directory.join(PROJECT_MANIFEST_NAME);
        if candidate.is_file() {
            return Ok(candidate);
        }
        if !directory.pop() {
            return Err(format!(
                "no {PROJECT_MANIFEST_NAME} found in the current directory or its ancestors"
            ));
        }
    }
}

fn execute_provider(provider: &ProjectDesignProvider, directory: &Path) -> Result<Vec<u8>, String> {
    let mut command = match provider.provider {
        ProjectProviderKind::Command => {
            let mut command = Command::new(&provider.command[0]);
            command.args(&provider.command[1..]);
            command
        }
        ProjectProviderKind::Cargo => {
            let mut command = Command::new("cargo");
            command.args([
                "run",
                "--quiet",
                "--package",
                provider
                    .package
                    .as_deref()
                    .expect("validated Cargo provider has a package"),
                "--bin",
                provider
                    .binary
                    .as_deref()
                    .expect("validated Cargo provider has a binary"),
            ]);
            if !provider.features.is_empty() {
                command.arg("--features").arg(provider.features.join(","));
            }
            command
        }
    };
    let output = command
        .current_dir(directory)
        .output()
        .map_err(|error| format!("cannot start provider: {error}"))?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!(
            "process exited with status {}; stderr: {}",
            output
                .status
                .code()
                .map_or_else(|| "signal".to_owned(), |code| code.to_string()),
            stderr.trim()
        ));
    }
    Ok(output.stdout)
}

fn emit_semantic_document(
    document: &SemanticDocument,
    output: Option<&Path>,
) -> Result<bool, String> {
    let json = document
        .to_json_pretty()
        .map_err(|error| format!("cannot encode semantic document: {error}"))?;
    if let Some(output) = output {
        write_file(output, json.as_bytes())?;
        println!("wrote {}", output.display());
    } else {
        println!("{json}");
    }
    Ok(true)
}

fn emit_bom(document: &SemanticDocument, output: Option<&Path>) -> Result<bool, String> {
    let layout = document
        .pcb
        .as_ref()
        .ok_or_else(|| "semantic document has no PCB layout".to_owned())?;
    let placement = layout.resolve_placement_constraints(&document.circuit);
    if !placement.is_satisfied() {
        return Err(format!(
            "placement constraints are unsatisfied ({} issues)",
            placement.issues.len()
        ));
    }
    let mut resolved = layout.clone();
    resolved.placements = placement.placements;
    let assembly = AssemblyOutputs::from_design(&document.circuit, &resolved)
        .ok_or_else(|| "cannot derive BOM from an invalid circuit layout".to_owned())?;
    let csv = assembly.bom_csv();
    if let Some(output) = output {
        write_file(output, csv.as_bytes())?;
        println!(
            "wrote {} with {} BOM line(s)",
            output.display(),
            assembly.bom.len()
        );
    } else {
        print!("{csv}");
    }
    Ok(assembly.unplaced_instances.is_empty())
}

fn check(loaded: &LoadedDesign) -> Result<bool, String> {
    if loaded.document.pcb.is_none() {
        let erc = loaded.document.circuit.electrical_rule_check();
        println!(
            "semantic document is valid; ERC reported {} issue(s); no PCB is attached",
            erc.issues.len()
        );
        return Ok(erc.issues.is_empty());
    }

    let report = loaded
        .document
        .prepare_release(loaded.release_options.clone())
        .map_err(|error| format!("cannot construct release evidence: {error}"))?;
    let blockers = report.release_blockers();
    println!(
        "semantic document is valid; {} release blocker(s), {} DRC violation(s), {} fabrication file(s)",
        blockers.len(),
        report.drc.violations.len(),
        report.fabrication.files.len()
    );
    for blocker in &blockers {
        println!("blocker: {blocker:?}");
    }
    Ok(blockers.is_empty())
}

fn export_kicad(document: &SemanticDocument, output: &Path) -> Result<bool, String> {
    let layout = document
        .pcb
        .as_ref()
        .ok_or_else(|| "semantic document has no PCB layout".to_owned())?;
    let report = layout
        .export_kicad(&document.circuit, KiCadExportOptions::default())
        .map_err(|error| format!("KiCad export failed: {error}"))?;
    write_file(output, report.board.as_bytes())?;
    if let Some(rules) = &report.design_rules {
        write_file(&output.with_extension("kicad_dru"), rules.as_bytes())?;
    }
    if let Some(project) = &report.project {
        write_file(&output.with_extension("kicad_pro"), project.as_bytes())?;
    }
    println!(
        "wrote {} with {} numeric projection(s) and {} typed omission(s)",
        output.display(),
        report.numeric_projections.len(),
        report.omissions.len()
    );
    Ok(report.omissions.is_empty())
}

fn export_svg(document: &SemanticDocument, output: &Path) -> Result<bool, String> {
    let layout = document
        .pcb
        .as_ref()
        .ok_or_else(|| "semantic document has no PCB layout".to_owned())?;
    let report = layout
        .to_svg(&document.circuit, PcbSvgOptions::default())
        .map_err(|error| format!("PCB SVG export failed: {error}"))?;
    write_file(output, report.svg.as_bytes())?;
    println!(
        "wrote {} with {} numeric projection(s)",
        output.display(),
        report.projections.len()
    );
    Ok(true)
}

fn release(loaded: &LoadedDesign, output: &Path) -> Result<bool, String> {
    let report = loaded
        .document
        .prepare_release(loaded.release_options.clone())
        .map_err(|error| format!("cannot construct release evidence: {error}"))?;
    fs::create_dir_all(output)
        .map_err(|error| format!("cannot create {}: {error}", output.display()))?;

    for file in &report.fabrication.files {
        let relative = safe_file_name(&file.name)?;
        write_file(&output.join(relative), &file.bytes)?;
    }
    write_file(
        &output.join("bom.csv"),
        report.assembly.bom_csv().as_bytes(),
    )?;
    write_file(
        &output.join("pick-and-place.csv"),
        report.assembly.pick_and_place_csv().as_bytes(),
    )?;
    write_file(
        &output.join("dnp.csv"),
        report.assembly.dnp_csv().as_bytes(),
    )?;

    let blockers = report.release_blockers();
    println!(
        "wrote {} fabrication and 3 assembly file(s) to {}; {} release blocker(s)",
        report.fabrication.files.len(),
        output.display(),
        blockers.len()
    );
    for blocker in &blockers {
        println!("blocker: {blocker:?}");
    }
    Ok(blockers.is_empty())
}

fn safe_file_name(name: &str) -> Result<PathBuf, String> {
    let path = Path::new(name);
    if path.components().count() == 1 && path.file_name().is_some() {
        Ok(path.to_owned())
    } else {
        Err(format!(
            "fabrication package proposed unsafe filename `{name}`"
        ))
    }
}

fn write_file(path: &Path, bytes: &[u8]) -> Result<(), String> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)
            .map_err(|error| format!("cannot create {}: {error}", parent.display()))?;
    }
    fs::write(path, bytes).map_err(|error| format!("cannot write {}: {error}", path.display()))
}
