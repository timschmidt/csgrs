use std::fs;
use std::path::{Path, PathBuf};
use std::process::{self, Command};
use std::time::{Duration, Instant};

use clap::Parser;
use hyperdrc::{Cli, run};

struct Fixture {
    label: &'static str,
    zip_path: &'static str,
    board_path: &'static str,
    gerber_dir: Option<&'static str>,
    edge_cuts: Option<&'static str>,
}

const FIXTURES: &[Fixture] = &[
    Fixture {
        label: "cparti-fpga",
        zip_path: "docs/CPArti FPGA dev board.zip",
        board_path: "CPArti FPGA dev board.kicad_pcb",
        gerber_dir: Some("gerbers"),
        edge_cuts: Some("CPArti FPGA dev board-Edge_Cuts.gbr"),
    },
    Fixture {
        label: "hvp109a",
        zip_path: "docs/HVP109A.zip",
        board_path: "HVP109A.kicad_pcb",
        gerber_dir: None,
        edge_cuts: None,
    },
];

fn main() {
    let workspace = std::env::temp_dir().join(format!("hyperdrc-fixture-smoke-{}", process::id()));
    let _ = fs::remove_dir_all(&workspace);
    fs::create_dir_all(&workspace).expect("fixture benchmark workspace should be creatable");

    let total_elapsed = time("fixture_smoke_total", || {
        let mut covered_kicad = 0usize;
        let mut covered_gerber = 0usize;

        for fixture in FIXTURES {
            let Some(package_dir) = extract_fixture(&workspace, fixture) else {
                continue;
            };
            covered_kicad += run_kicad_smoke(fixture, &package_dir);
            covered_gerber += run_gerber_smoke(fixture, &package_dir);
        }

        assert!(
            covered_kicad == FIXTURES.len(),
            "fixture smoke benchmark covered {covered_kicad} KiCad board(s), expected {}",
            FIXTURES.len()
        );
        assert!(
            covered_gerber > 0,
            "fixture smoke benchmark did not cover any Gerber package"
        );
    });

    let _ = fs::remove_dir_all(&workspace);
    assert!(
        total_elapsed < Duration::from_secs(120),
        "fixture smoke benchmark took {total_elapsed:?}"
    );
}

fn extract_fixture(workspace: &Path, fixture: &Fixture) -> Option<PathBuf> {
    let zip_path = Path::new(env!("CARGO_MANIFEST_DIR")).join(fixture.zip_path);
    if !zip_path.exists() {
        eprintln!(
            "fixture_smoke: skipping {} because {} is missing",
            fixture.label,
            zip_path.display()
        );
        return None;
    }

    let package_dir = workspace.join(fixture.label);
    fs::create_dir_all(&package_dir).expect("fixture package directory should be creatable");
    let output = Command::new("unzip")
        .arg("-q")
        .arg("-o")
        .arg(&zip_path)
        .arg("-d")
        .arg(&package_dir)
        .output()
        .unwrap_or_else(|error| {
            panic!(
                "fixture_smoke: failed to run unzip for {}: {error}",
                zip_path.display()
            )
        });
    assert!(
        output.status.success(),
        "fixture_smoke: unzip failed for {}: {}",
        zip_path.display(),
        String::from_utf8_lossy(&output.stderr)
    );
    Some(package_dir)
}

fn run_kicad_smoke(fixture: &Fixture, package_dir: &Path) -> usize {
    let pcb_path = package_dir.join(fixture.board_path);
    assert!(
        pcb_path.exists(),
        "fixture_smoke: missing extracted board {}",
        pcb_path.display()
    );

    time(&format!("fixture_{}_kicad_smoke", fixture.label), || {
        let args = vec![
            "hyperdrc".to_string(),
            "--kicad-pcb".to_string(),
            pcb_path.display().to_string(),
            "--check".to_string(),
            "layer-sanity".to_string(),
            "--check".to_string(),
            "board-outline-sanity".to_string(),
            "--check".to_string(),
            "min-copper-neck".to_string(),
            "--check".to_string(),
            "drill-spacing".to_string(),
            "--min-width".to_string(),
            "0.0762".to_string(),
            "--format".to_string(),
            "text".to_string(),
        ];
        let outcome = run(Cli::parse_from(args)).unwrap_or_else(|error| {
            panic!(
                "fixture_smoke: KiCad smoke run failed for {}: {error}",
                pcb_path.display()
            )
        });
        assert!(!outcome.report.inputs.is_empty());
    });
    1
}

fn run_gerber_smoke(fixture: &Fixture, package_dir: &Path) -> usize {
    let Some(gerber_dir_name) = fixture.gerber_dir else {
        return 0;
    };
    let Some(edge_cuts_name) = fixture.edge_cuts else {
        return 0;
    };

    let gerber_dir = package_dir.join(gerber_dir_name);
    assert!(
        gerber_dir.exists(),
        "fixture_smoke: missing extracted Gerber directory {}",
        gerber_dir.display()
    );
    let smoke_dir = package_dir.join("gerber-smoke-subset");
    fs::create_dir_all(&smoke_dir).expect("Gerber smoke directory should be creatable");
    fs::copy(
        gerber_dir.join(edge_cuts_name),
        smoke_dir.join(edge_cuts_name),
    )
    .unwrap_or_else(|error| panic!("fixture_smoke: failed to copy edge cuts Gerber: {error}"));

    time(&format!("fixture_{}_gerber_smoke", fixture.label), || {
        let args = vec![
            "hyperdrc".to_string(),
            "--gerber-dir".to_string(),
            smoke_dir.display().to_string(),
            "--check".to_string(),
            "layer-sanity".to_string(),
            "--min-width".to_string(),
            "0.0762".to_string(),
            "--format".to_string(),
            "text".to_string(),
        ];
        let outcome = run(Cli::parse_from(args)).unwrap_or_else(|error| {
            panic!(
                "fixture_smoke: Gerber smoke run failed for {}: {error}",
                smoke_dir.display()
            )
        });
        assert!(!outcome.report.inputs.is_empty());
    });
    1
}

fn time(label: &str, work: impl FnOnce()) -> Duration {
    let started = Instant::now();
    work();
    let elapsed = started.elapsed();
    println!("{label} ms={:.3}", elapsed.as_secs_f64() * 1000.0);
    elapsed
}
