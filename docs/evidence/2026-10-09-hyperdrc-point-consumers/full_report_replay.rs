use clap::Parser;
use hyperdrc::{Cli, run};
use std::path::PathBuf;

fn main() {
    let mut args = std::env::args().skip(1);
    let inputs = PathBuf::from(args.next().unwrap());
    let outputs = PathBuf::from(args.next().unwrap());
    assert!(args.next().is_none());
    std::fs::create_dir_all(&outputs).unwrap();
    for (label, filename) in [
        ("cparti-fpga", "CPArti FPGA dev board.kicad_pcb"),
        ("hvp109a", "HVP109A.kicad_pcb"),
    ] {
        let mut command = vec![
            "hyperdrc".to_string(),
            "--kicad-pcb".to_string(),
            inputs.join(label).join(filename).display().to_string(),
        ];
        for check in [
            "edge-stitching-readiness",
            "chassis-stitching-readiness",
            "return-path-readiness",
            "power-via-array-readiness",
            "differential-pair-via-proximity-readiness",
            "differential-pair-via-return-readiness",
            "mouse-bite-readiness",
        ] {
            command.extend(["--check".to_string(), check.to_string()]);
        }
        let outcome = run(Cli::parse_from(command)).unwrap();
        assert!(!outcome.report.inputs.is_empty());
        std::fs::write(
            outputs.join(format!("{label}.json")),
            serde_json::to_vec(&outcome.report).unwrap(),
        )
        .unwrap();
    }
}
