#![cfg(all(feature = "drc", feature = "interchange"))]

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

use hypercircuit::{
    BoardId, CircuitId, DrcReadinessPolicy, HyperDrcHandoff, KiCadImportOmission,
    KiCadImportOptions, KiCadImportReport, MaterializationOptions, Real, SemanticDocument,
};
use serde::Deserialize;
use sha2::{Digest, Sha256};

#[derive(Debug, Deserialize)]
struct EasyduinoManifest {
    repository: String,
    commit: String,
    license: String,
    license_sha256: String,
    boards: Vec<BoardSpec>,
}

#[derive(Clone, Debug, Deserialize)]
struct BoardSpec {
    slug: String,
    title: String,
    upstream_path: String,
    pcb_sha256: String,
    project_sha256: String,
    native_sha256: String,
    nets: usize,
    instances: usize,
    land_patterns: usize,
    pads: usize,
    placements: usize,
    routes: usize,
    vias: usize,
    zones: usize,
    stackup_layers: usize,
    net_classes: usize,
    fast_drc_findings: usize,
    fast_drc_handoff_omissions: usize,
}

fn fixture_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/easyduino")
}

fn manifest() -> EasyduinoManifest {
    toml::from_str(
        &fs::read_to_string(fixture_root().join("manifest.toml"))
            .expect("read Easyduino fixture manifest"),
    )
    .expect("decode Easyduino fixture manifest")
}

fn spec(slug: &str) -> BoardSpec {
    manifest()
        .boards
        .into_iter()
        .find(|board| board.slug == slug)
        .unwrap_or_else(|| panic!("Easyduino manifest is missing {slug}"))
}

fn sha256(path: &Path) -> String {
    format!(
        "{:x}",
        Sha256::digest(
            fs::read(path)
                .unwrap_or_else(|error| { panic!("read {} for hashing: {error}", path.display()) })
        )
    )
}

fn import(board: &BoardSpec) -> KiCadImportReport {
    let source = fixture_root().join("upstream").join(&board.slug);
    KiCadImportReport::from_project_paths(
        &source.join("board.kicad_pcb"),
        &source.join("board.kicad_pro"),
        None,
        KiCadImportOptions::new(
            CircuitId::new(format!("easyduino-{}", board.slug)).unwrap(),
            BoardId::new(format!("easyduino-{}", board.slug)).unwrap(),
            (Real::from(35) / Real::from(1_000)).unwrap(),
        ),
    )
    .unwrap_or_else(|error| panic!("import Easyduino {}: {error}", board.slug))
}

fn native(board: &BoardSpec) -> SemanticDocument {
    let path = fixture_root()
        .join("native")
        .join(format!("{}.hypercircuit.json", board.slug));
    SemanticDocument::from_json(
        &fs::read_to_string(&path)
            .unwrap_or_else(|error| panic!("read {}: {error}", path.display())),
    )
    .unwrap_or_else(|error| panic!("decode native Easyduino {}: {error}", board.slug))
}

fn assert_counts(board: &BoardSpec, document: &SemanticDocument) {
    let layout = document.pcb.as_ref().expect("Easyduino fixture has a PCB");
    assert_eq!(
        document.circuit.nets.len(),
        board.nets,
        "{} nets",
        board.slug
    );
    assert_eq!(
        document.circuit.instances.len(),
        board.instances,
        "{} instances",
        board.slug
    );
    assert_eq!(
        layout.land_patterns.len(),
        board.land_patterns,
        "{} land patterns",
        board.slug
    );
    assert_eq!(
        layout
            .land_patterns
            .iter()
            .map(|pattern| pattern.pads.len())
            .sum::<usize>(),
        board.pads,
        "{} pads",
        board.slug
    );
    assert_eq!(
        layout.placements.len(),
        board.placements,
        "{} placements",
        board.slug
    );
    assert_eq!(layout.routes.len(), board.routes, "{} routes", board.slug);
    assert_eq!(layout.vias.len(), board.vias, "{} vias", board.slug);
    assert_eq!(layout.zones.len(), board.zones, "{} zones", board.slug);
    assert_eq!(
        layout.stackup.layers.len(),
        board.stackup_layers,
        "{} stackup layers",
        board.slug
    );
    assert_eq!(
        layout.rules.net_classes.len(),
        board.net_classes,
        "{} net classes",
        board.slug
    );
}

#[test]
fn manifest_pins_the_complete_easyduino_repository_inventory() {
    let manifest = manifest();
    assert_eq!(manifest.repository, "https://github.com/Hanqaqa/Easyduino");
    assert_eq!(manifest.commit, "53b14b66d64f25c55f88971c1c1b51656126bd30");
    assert_eq!(manifest.license, "CERN-OHL-P-2.0");
    assert_eq!(manifest.boards.len(), 6);
    assert_eq!(
        sha256(&fixture_root().join("UPSTREAM_LICENSE.txt")),
        manifest.license_sha256
    );

    let expected = manifest
        .boards
        .iter()
        .map(|board| board.slug.clone())
        .collect::<BTreeSet<_>>();
    assert_eq!(
        expected,
        BTreeSet::from([
            "nano".into(),
            "uno".into(),
            "esp32".into(),
            "esp32s3".into(),
            "rp2040".into(),
            "stm32f103".into(),
        ])
    );

    let upstream = fs::read_dir(fixture_root().join("upstream"))
        .unwrap()
        .map(|entry| entry.unwrap().file_name().to_string_lossy().into_owned())
        .collect::<BTreeSet<_>>();
    assert_eq!(upstream, expected);
    let native = fs::read_dir(fixture_root().join("native"))
        .unwrap()
        .map(|entry| {
            entry
                .unwrap()
                .file_name()
                .to_string_lossy()
                .trim_end_matches(".hypercircuit.json")
                .to_owned()
        })
        .collect::<BTreeSet<_>>();
    assert_eq!(native, expected);

    for board in &manifest.boards {
        assert!(!board.title.is_empty());
        assert!(!board.upstream_path.is_empty());
        assert!(
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("examples")
                .join(format!("easyduino_{}.rs", board.slug))
                .is_file(),
            "native example is missing for {}",
            board.slug
        );
    }
}

#[test]
fn pinned_kicad_projects_import_to_exact_native_semantic_fixtures() {
    for board in manifest().boards {
        let source = fixture_root().join("upstream").join(&board.slug);
        let native_path = fixture_root()
            .join("native")
            .join(format!("{}.hypercircuit.json", board.slug));
        assert_eq!(sha256(&source.join("board.kicad_pcb")), board.pcb_sha256);
        assert_eq!(
            sha256(&source.join("board.kicad_pro")),
            board.project_sha256
        );
        assert_eq!(sha256(&native_path), board.native_sha256);

        let imported = import(&board);
        assert!(
            imported.circuit.validate().is_valid(),
            "{} imported circuit",
            board.slug
        );
        assert!(
            imported.layout.validate(&imported.circuit).is_valid(),
            "{} imported layout",
            board.slug
        );
        assert!(!imported.numeric_imports.is_empty());
        assert!(
            !imported
                .omissions
                .iter()
                .any(|omission| matches!(omission, KiCadImportOmission::UnconnectedCopper { .. })),
            "{} lost named KiCad copper connectivity: {:?}",
            board.slug,
            imported.omissions
        );

        let imported_document = SemanticDocument::new(imported.circuit, None)
            .unwrap()
            .with_pcb(imported.layout)
            .unwrap();
        let native_document = native(&board);
        assert_counts(&board, &native_document);
        assert_eq!(
            native_document, imported_document,
            "{} native fixture diverged from its pinned KiCad import",
            board.slug
        );
    }
}

fn run_fast_hyperdrc(slug: &str) {
    let board = spec(slug);
    let document = native(&board);
    let mut layout = document.pcb.as_ref().unwrap().clone();
    assert_eq!(layout.zones.len(), board.zones);

    // Authored zones are parity-tested above. The explicit full benchmark
    // retains and re-pours them; routine DRC covers every independently
    // authored pad, route, via, drill, placement, rule, and stackup fact.
    layout.zones.clear();
    let materialized = layout
        .materialize(
            &document.circuit,
            MaterializationOptions {
                circular_segments: 8,
                aggregate_layer_images: false,
                ..MaterializationOptions::default()
            },
        )
        .unwrap_or_else(|error| panic!("materialize Easyduino {slug}: {error}"));
    assert!(!materialized.layer_images_aggregated);
    assert!(materialized.copper_layers.is_empty());
    assert!(materialized.process_layers.is_empty());
    assert!(materialized.copper_features.len() >= board.routes + board.vias);

    let handoff = HyperDrcHandoff::from_materialization(&layout, &materialized);
    assert_eq!(
        handoff.omissions.len(),
        board.fast_drc_handoff_omissions,
        "{slug} HyperDRC handoff omissions"
    );
    let readiness = handoff.run_readiness(&DrcReadinessPolicy::default());
    assert_eq!(
        readiness.violations.len(),
        board.fast_drc_findings,
        "{slug} HyperDRC findings"
    );
}

#[test]
fn nano_full_fidelity_exact_aggregate_has_no_layer_blockers() {
    let board = spec("nano");
    let document = native(&board);
    let mut layout = document.pcb.as_ref().unwrap().clone();
    let placement = layout.resolve_placement_constraints(&document.circuit);
    assert!(placement.is_satisfied());
    layout.placements = placement.placements;
    let materialized = layout
        .materialize(&document.circuit, MaterializationOptions::default())
        .expect("full-fidelity Nano materialization must complete");
    assert!(materialized.layer_images_aggregated);
    assert!(
        materialized
            .copper_layers
            .iter()
            .all(|layer| layer.blocker.is_none()),
        "exact copper aggregate retained a blocker: {:#?}",
        materialized.copper_layers
    );
    assert!(
        materialized
            .process_layers
            .iter()
            .all(|layer| layer.blocker.is_none()),
        "exact process aggregate retained a blocker: {:#?}",
        materialized.process_layers
    );
}

macro_rules! easyduino_hyperdrc_test {
    ($name:ident, $slug:literal) => {
        #[test]
        fn $name() {
            run_fast_hyperdrc($slug);
        }
    };
}

easyduino_hyperdrc_test!(nano_runs_end_to_end_through_hyperdrc, "nano");
easyduino_hyperdrc_test!(uno_runs_end_to_end_through_hyperdrc, "uno");
easyduino_hyperdrc_test!(esp32_runs_end_to_end_through_hyperdrc, "esp32");
easyduino_hyperdrc_test!(esp32s3_runs_end_to_end_through_hyperdrc, "esp32s3");
easyduino_hyperdrc_test!(rp2040_runs_end_to_end_through_hyperdrc, "rp2040");
easyduino_hyperdrc_test!(stm32f103_runs_end_to_end_through_hyperdrc, "stm32f103");
