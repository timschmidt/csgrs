//! Tests that load and validate adversarial fixture descriptions.

use std::fs;
use std::path::Path;

#[test]
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
