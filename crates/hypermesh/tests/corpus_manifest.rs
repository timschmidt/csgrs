#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod support;

use std::collections::BTreeSet;
use std::path::Path;

use toml::Value;

fn manifest() -> Value {
    toml::from_str(include_str!("../benchmarks/corpus/fixtures.toml"))
        .expect("fixture manifest must be valid TOML")
}

fn migration_ledger() -> Value {
    toml::from_str(include_str!(
        "../benchmarks/corpus/implementation-test-migration.toml"
    ))
    .expect("implementation-test migration ledger must be valid TOML")
}

fn string_array<'a>(fixture: &'a Value, field: &str) -> Vec<&'a str> {
    fixture[field]
        .as_array()
        .unwrap_or_else(|| panic!("fixture field {field} must be an array"))
        .iter()
        .map(|value| {
            value
                .as_str()
                .unwrap_or_else(|| panic!("fixture field {field} must contain strings"))
        })
        .collect()
}

#[test]
fn fixture_manifest_is_complete_unique_and_reproducible() {
    let manifest = manifest();
    assert_eq!(manifest["schema"].as_integer(), Some(1));
    assert_eq!(manifest["admission"].as_str(), Some("monotonic"));
    let fixtures = manifest["fixture"]
        .as_array()
        .expect("fixture manifest must contain [[fixture]] records");
    assert!(fixtures.len() >= 16, "fixture registry unexpectedly shrank");

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut ids = BTreeSet::new();
    for fixture in fixtures {
        let id = fixture["id"].as_str().expect("fixture requires string id");
        assert!(ids.insert(id), "duplicate fixture id {id}");
        for field in [
            "status",
            "provenance",
            "license",
            "source_kind",
            "coordinate_class",
            "size",
            "expected_certainty",
        ] {
            assert!(
                fixture[field]
                    .as_str()
                    .is_some_and(|value| !value.is_empty()),
                "fixture {id} requires nonempty {field}"
            );
        }
        for field in ["topology", "operations", "policies", "tiers"] {
            assert!(
                !string_array(fixture, field).is_empty(),
                "fixture {id} requires at least one {field} value"
            );
        }
        assert!(
            fixture["cgal_eligible"].as_bool().is_some(),
            "fixture {id} requires cgal_eligible"
        );
        if fixture["source_kind"].as_str() == Some("exact-asset-pair") {
            for side in ["left", "right"] {
                let relative = fixture[side]
                    .as_str()
                    .unwrap_or_else(|| panic!("exact fixture {id} requires {side}"));
                assert!(
                    root.join(relative).is_file(),
                    "exact fixture {id} is missing {relative}"
                );
            }
        }
        if string_array(fixture, "tiers").contains(&"heap") {
            assert!(
                matches!(fixture["size"].as_str(), Some("large" | "xl")),
                "heap fixture {id} must be large or xl"
            );
        }
    }
}

#[test]
fn every_in_process_competitive_case_has_a_manifest_record() {
    let manifest = manifest();
    let registered = manifest["fixture"]
        .as_array()
        .expect("fixture records")
        .iter()
        .map(|fixture| fixture["id"].as_str().expect("fixture id"))
        .collect::<BTreeSet<_>>();
    for case in support::corpus() {
        assert!(
            registered.contains(case.name),
            "competitive case {} is absent from fixtures.toml",
            case.name
        );
    }
    assert!(registered.contains("subdivided_overlapping_boxes_3072_each"));
}

#[test]
fn corpus_spans_initial_replacement_path_classes() {
    let manifest = manifest();
    let tags = manifest["fixture"]
        .as_array()
        .expect("fixture records")
        .iter()
        .flat_map(|fixture| string_array(fixture, "topology"))
        .collect::<BTreeSet<_>>();
    for required in [
        "disjoint",
        "strict-containment",
        "identical",
        "coplanar-overlay",
        "full-face-contact",
        "partial-face-contact",
        "transverse-face-intersection",
        "transverse-edge-face-intersection",
        "concave",
        "multi-component",
        "high-genus",
        "nonconvex",
        "terminal-equality",
        "empty-result",
        "transverse-point",
        "transverse-segment",
        "shared-feature",
        "coplanar-disjoint",
        "coplanar-vertex-contact",
        "coplanar-edge-contact",
        "orientation-inversion",
        "negative-winding-cavity",
        "genus-one",
        "self-intersecting-pwn",
        "winding-multiplicity-two",
        "exact-embedding",
        "reflection",
        "face-permutation",
        "operand-permutation",
        "high-operand-count",
        "batched-expression",
    ] {
        assert!(tags.contains(required), "fixture topology gap: {required}");
    }
}

#[test]
fn exact_pairwise_microcases_cover_every_public_intersection_variant() {
    let manifest = manifest();
    let fixture = manifest["fixture"]
        .as_array()
        .expect("fixture records")
        .iter()
        .find(|fixture| fixture["id"].as_str() == Some("exact_pairwise_intersection_microcases"))
        .expect("exact pairwise intersection microcase fixture");

    assert_eq!(fixture["case_count"].as_integer(), Some(12));
    assert_eq!(
        fixture["orientation_order_variants_per_case"].as_integer(),
        Some(8)
    );
    assert_eq!(
        fixture["exact_rectangle_oracle_cases"].as_integer(),
        Some(256)
    );
    assert_eq!(
        fixture["rectangle_orientation_order_variants_per_case"].as_integer(),
        Some(8)
    );
    assert_eq!(
        string_array(fixture, "expected_pairwise_classes"),
        [
            "Disjoint",
            "NonCoplanarPoint",
            "NonCoplanarSegment",
            "CoplanarPoint",
            "CoplanarSegment",
            "CoplanarOverlap",
        ]
    );
}

#[test]
fn large_box_heap_fixture_covers_certified_and_general_paths() {
    let manifest = manifest();
    let fixture = manifest["fixture"]
        .as_array()
        .expect("fixture records")
        .iter()
        .find(|fixture| fixture["id"].as_str() == Some("subdivided_overlapping_boxes_3072_each"))
        .expect("large subdivided-box heap fixture");

    assert_eq!(
        string_array(fixture, "heap_probe_modes"),
        ["boxes-3072", "boxes-3072-general"]
    );
    assert_eq!(fixture["input_triangles"].as_integer(), Some(6144));
}

#[test]
fn removed_engine_tests_have_complete_pinned_migration_coverage() {
    let ledger = migration_ledger();
    assert_eq!(ledger["schema"].as_integer(), Some(1));
    assert_eq!(ledger["compatibility_code"].as_bool(), Some(false));
    assert_eq!(
        ledger["source_commit"].as_str(),
        Some("f56371ec7eda83518c3960792a42a27a5634f2a4")
    );

    let mappings = ledger["mapping"]
        .as_array()
        .expect("migration ledger requires [[mapping]] records");
    let mapping_ids = mappings
        .iter()
        .map(|mapping| mapping["id"].as_str().expect("mapping requires id"))
        .collect::<BTreeSet<_>>();
    assert_eq!(
        mapping_ids.len(),
        mappings.len(),
        "duplicate migration mapping"
    );

    let sources = ledger["removed_source"]
        .as_array()
        .expect("migration ledger requires [[removed_source]] records");
    assert_eq!(sources.len(), 8);
    assert_eq!(
        sources
            .iter()
            .map(|source| source["test_count"].as_integer().unwrap())
            .sum::<i64>(),
        1_113
    );
    for source in sources {
        let path = source["path"].as_str().expect("removed source path");
        let hash = source["sha256"].as_str().expect("removed source hash");
        assert_eq!(hash.len(), 64, "invalid source hash for {path}");
        let catch_all = source["catch_all_mapping"]
            .as_str()
            .expect("removed source catch-all mapping");
        assert!(
            mapping_ids.contains(catch_all),
            "removed source {path} has unknown catch-all mapping {catch_all}"
        );
    }

    for mapping in mappings {
        let id = mapping["id"].as_str().expect("mapping id");
        assert!(
            !string_array(mapping, "replacement_tests").is_empty(),
            "mapping {id} must name current invariant tests"
        );
        assert!(
            !string_array(mapping, "fixture_ids").is_empty(),
            "mapping {id} must name permanent fixtures"
        );
    }
}
