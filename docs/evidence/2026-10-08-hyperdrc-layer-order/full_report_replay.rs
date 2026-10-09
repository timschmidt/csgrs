use clap::Parser;
use hyperdrc::{Cli, run};
use std::path::PathBuf;

fn main() {
    let mut args = std::env::args().skip(1);
    let inputs = PathBuf::from(args.next().unwrap());
    let outputs = PathBuf::from(args.next().unwrap());
    assert!(args.next().is_none());
    std::fs::create_dir_all(&outputs).unwrap();
    for (label, source_flag, path, checks) in [
        (
            "cparti-fpga",
            "--kicad-pcb",
            inputs.join("cparti-fpga/CPArti FPGA dev board.kicad_pcb"),
            vec![
                "layer-sanity",
                "board-outline-sanity",
                "min-copper-neck",
                "drill-spacing",
            ],
        ),
        (
            "hvp109a",
            "--kicad-pcb",
            inputs.join("hvp109a/HVP109A.kicad_pcb"),
            vec![
                "layer-sanity",
                "board-outline-sanity",
                "min-copper-neck",
                "drill-spacing",
            ],
        ),
        (
            "cparti-gerber",
            "--gerber-dir",
            inputs.join("cparti-fpga/gerber-smoke-subset"),
            vec!["layer-sanity"],
        ),
    ] {
        let mut command = vec![
            "hyperdrc".to_string(),
            source_flag.to_string(),
            path.display().to_string(),
        ];
        for check in checks {
            command.extend(["--check".to_string(), check.to_string()]);
        }
        command.extend(["--min-width", "0.0762", "--format", "text"].map(str::to_string));
        let outcome = run(Cli::parse_from(command)).unwrap();
        assert!(!outcome.report.inputs.is_empty());
        std::fs::write(
            outputs.join(format!("{label}.json")),
            serde_json::to_vec(&outcome.report).unwrap(),
        )
        .unwrap();
    }
}
