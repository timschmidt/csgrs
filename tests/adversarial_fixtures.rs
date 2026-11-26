//! Lexical metadata checks for recorded adversarial fixture descriptions.
//! Geometry, feature selections and expected/observed outcomes are not replayed here.

use std::fs;
use std::path::Path;

#[test]
/// Requires a nonempty directory and six literal labels in each immediate `.txt` entry.
/// Sorts paths for deterministic checking; does not parse field values or require
/// that at least one entry is a text fixture. I/O errors panic with test diagnostics.
fn adversarial_regression_fixtures_are_recorded() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/adversarial");
    let mut entries = fs::read_dir(&root)
        .unwrap_or_else(|err| panic!("failed to read {}: {err}", root.display()))
        .collect::<Result<Vec<_>, _>>()
        .unwrap();
    entries.sort_by_key(|entry| entry.path());
    assert!(
        !entries.is_empty(),
        "expected adversarial regression fixtures"
    );

    for entry in entries {
        let path = entry.path();
        if path.extension().and_then(|ext| ext.to_str()) != Some("txt") {
            continue;
        }
        let text = fs::read_to_string(&path).unwrap();
        for required in [
            "feature_flags:",
            "real_type:",
            "operation_sequence:",
            "expected_behavior:",
            "observed_behavior:",
            "normalized_geometry_summary:",
        ] {
            assert!(
                text.contains(required),
                "{} missing required fixture field {required}",
                path.display()
            );
        }
    }
}
