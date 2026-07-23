//! Regression checks for the geometry/circuit ownership boundary.

use std::{fs, path::Path};

#[test]
fn public_rust_vocabulary_does_not_reintroduce_circuit_or_pcb_types() {
    let manifest = include_str!("../../Cargo.toml");
    assert!(
        !manifest.lines().any(|line| {
            line.trim_start().starts_with("hypercircuit ")
                || line.trim_start().starts_with("hypercircuit=")
        }),
        "the geometry engine must not depend on hypercircuit"
    );

    let mut sources = Vec::new();
    collect_rust_sources(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("src").as_path(),
        &mut sources,
    );
    let forbidden = [
        "Circuit",
        "Net",
        "NetClass",
        "Device",
        "DevicePin",
        "Footprint",
        "LandPattern",
        "Pad",
        "Via",
        "Trace",
        "Route",
        "Board",
        "Stackup",
        "CopperZone",
        "Keepout",
        "Placement",
        "Schematic",
        "PinBinding",
    ];

    for source in sources {
        let text = fs::read_to_string(&source).expect("Rust source is readable");
        for kind in ["struct", "enum", "trait", "type"] {
            for name in forbidden {
                let declaration = format!("pub {kind} {name}");
                assert!(
                    !text.contains(&declaration),
                    "{} declares prohibited domain vocabulary `{declaration}`",
                    source.display()
                );
            }
        }
    }
}

#[test]
fn the_only_electronics_markers_are_the_two_deprecated_compatibility_variants() {
    let metadata = include_str!("../parts/metadata.rs");
    assert_eq!(
        metadata
            .matches("compatibility marker scheduled for removal in csgrs 0.25.0")
            .count(),
        2
    );
    assert_eq!(metadata.matches("\n    Package,").count(), 1);
    assert_eq!(metadata.matches("\n    Electrical,").count(), 1);
}

fn collect_rust_sources(directory: &Path, output: &mut Vec<std::path::PathBuf>) {
    for entry in fs::read_dir(directory).expect("source directory is readable") {
        let path = entry.expect("source directory entry is readable").path();
        if path.is_dir() {
            collect_rust_sources(&path, output);
        } else if path.extension().is_some_and(|extension| extension == "rs") {
            output.push(path);
        }
    }
}
