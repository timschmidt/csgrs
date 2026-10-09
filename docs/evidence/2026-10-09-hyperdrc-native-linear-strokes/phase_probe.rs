//! Untimed witnesses plus wall-time breakdown of full-board layer/neck work.
//! This diagnostic does not replace either repository performance gate.

use hyperdrc::{Scalar, checks, kicad, report::Violation};
use std::{path::Path, time::Instant};

fn phase<T>(board: &str, layer: &str, name: &str, operation: impl FnOnce() -> T) -> T {
    let start = Instant::now();
    let result = operation();
    println!(
        "phase\t{board}\t{layer}\t{name}\t{}",
        start.elapsed().as_nanos()
    );
    result
}

fn main() {
    let min_area: Scalar = "1e-9".parse().unwrap();
    let min_width: Scalar = "0.0762".parse().unwrap();
    for input in std::env::args().skip(1) {
        let board = phase(&input, "", "load", || {
            kicad::load_kicad_pcb(Path::new(&input)).unwrap()
        });
        let layers = phase(&input, "", "copper_layers", || {
            board.copper_layers(&[]).unwrap()
        });
        let mut findings: Vec<Violation> = Vec::new();
        for (layer, copper) in &layers {
            let name = format!("{}:{layer}", board.source);
            findings.extend(phase(&input, layer, "layer_sanity", || {
                checks::layer_sanity(&name, copper, None)
            }));
            findings.extend(phase(&input, layer, "tiny", || {
                checks::tiny_layer_feature_readiness(&name, copper, &min_area)
            }));
            findings.extend(phase(&input, layer, "skinny", || {
                checks::skinny_layer_feature_readiness(&name, copper, &min_width, &min_area)
            }));
            findings.extend(phase(&input, layer, "duplicate_islands", || {
                checks::duplicate_layer_island_readiness(&name, copper, &min_area)
            }));
        }
        let named = layers
            .iter()
            .map(|(name, copper)| (format!("{}:{name}", board.source), copper))
            .collect::<Vec<_>>();
        findings.extend(phase(&input, "", "duplicate_layers", || {
            checks::duplicate_layer_geometry_readiness(&named, &min_area)
        }));
        for (layer, copper) in &layers {
            let name = format!("{}:{layer}", board.source);
            findings.extend(phase(&input, layer, "min_copper_neck", || {
                checks::min_copper_neck_width(&name, copper, &min_width, &min_area)
            }));
        }
        // Retain every finding; serialization runs after the measured checks.
        println!("findings\t{}", serde_json::to_string(&findings).unwrap());
    }
}
