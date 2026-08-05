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
    assert!(fixtures.len() >= 38, "fixture registry unexpectedly shrank");

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
            assert!(
                !string_array(fixture, "heap_probe_modes").is_empty(),
                "heap fixture {id} requires at least one process/kernel probe mode"
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
    for case in support::lower_dimensional_contact_corpus() {
        assert!(
            registered.contains(case.name),
            "closed-PWN contact case {} is absent from fixtures.toml",
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
        "source-edge-split-propagation",
        "intersection-curve-continuation",
        "edge-contact",
        "vertex-contact",
        "tangent-containment",
        "nonmanifold-output",
        "exact-symmetry-plane",
        "large-mesh",
        "cross-operand-coplanar-overlay",
        "opposite-face-diagonals",
        "fixed-coordinate-complexity",
        "exact-similarity",
        "fixed-topology-scalar-scaling",
        "wide-rational",
        "multi-shell-input",
        "disconnected-components",
        "sparse-broad-phase",
        "component-scaling",
        "fixed-local-topology",
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
fn every_large_heap_fixture_has_a_distinct_probe_selector() {
    let manifest = manifest();
    let mut selectors = BTreeSet::new();
    for fixture in manifest["fixture"]
        .as_array()
        .expect("fixture records")
        .iter()
        .filter(|fixture| string_array(fixture, "tiers").contains(&"heap"))
    {
        let id = fixture["id"].as_str().expect("fixture id");
        for selector in string_array(fixture, "heap_probe_modes") {
            assert!(
                selectors.insert(selector),
                "heap probe selector {selector} is duplicated by fixture {id}"
            );
        }
    }
    assert_eq!(
        selectors,
        [
            "boxes-3072",
            "boxes-3072-general",
            "dense-coplanar-16",
            "dense-coplanar-32",
            "sparse-shells-512",
            "voxel-torus-33",
            "voxel-torus-65",
            "wide-rational-64",
            "wide-rational-512",
            "wide-rational-2048",
            "yeahright",
            "yeahright-4",
            "yeahright-8",
            "yeahright-full-rotated",
        ]
        .into_iter()
        .collect()
    );
}

#[test]
fn sparse_multishell_family_scales_components_at_fixed_local_topology() {
    let manifest = manifest();
    let family = manifest["fixture"]
        .as_array()
        .expect("fixture records")
        .iter()
        .filter(|fixture| {
            fixture.get("scaling_family").and_then(Value::as_str)
                == Some("sparse_multishell_tetrahedra")
        })
        .collect::<Vec<_>>();
    assert_eq!(family.len(), support::SPARSE_MULTISHELL_COUNTS.len());
    assert_eq!(
        family
            .iter()
            .map(|fixture| fixture["scale_parameter"].as_integer().unwrap())
            .collect::<Vec<_>>(),
        support::SPARSE_MULTISHELL_COUNTS.map(|count| count as i64)
    );

    for shell_count in support::SPARSE_MULTISHELL_COUNTS {
        let case = support::sparse_multishell_tetrahedra_case(shell_count);
        assert_eq!(
            case.left.triangles.len() + case.right.triangles.len(),
            shell_count * 8
        );
        assert_eq!(
            case.left.positions.len() + case.right.positions.len(),
            shell_count * 8
        );
        for mesh in [&case.left, &case.right] {
            let summary = support::summarize(mesh);
            assert!(summary.closed);
            assert!(summary.nondegenerate);
            assert_eq!(summary.components, shell_count);
            assert_eq!(summary.triangles, shell_count * 4);
            assert_eq!(summary.vertices, shell_count * 4);
            assert_eq!(summary.volume, shell_count as f64 * 64.0 / 6.0);
            assert!(
                mesh.positions
                    .iter()
                    .flatten()
                    .all(|coordinate| coordinate.fract() == 0.0)
            );
        }

        let fixture = family
            .iter()
            .find(|fixture| fixture["scale_parameter"].as_integer() == Some(shell_count as i64))
            .expect("every generated shell count is manifested");
        assert_eq!(
            fixture["input_triangles"].as_integer(),
            Some((shell_count * 8) as i64)
        );
        assert_eq!(
            fixture["expected_output_triangles"]
                .as_array()
                .expect("sparse shell outputs are manifested")
                .iter()
                .map(|value| value.as_integer().unwrap())
                .collect::<Vec<_>>(),
            [
                shell_count * 16,
                shell_count * 4,
                shell_count * 12,
                shell_count * 8,
                shell_count * 20,
            ]
            .map(|count| count as i64)
        );
    }
}

#[test]
fn wide_rational_family_scales_scalar_width_without_changing_topology() {
    let manifest = manifest();
    let family = manifest["fixture"]
        .as_array()
        .expect("fixture records")
        .iter()
        .filter(|fixture| {
            fixture.get("scaling_family").and_then(Value::as_str) == Some("wide_rational_boxes")
        })
        .collect::<Vec<_>>();
    assert_eq!(family.len(), support::WIDE_RATIONAL_SHIFTS.len() + 1);
    assert_eq!(
        family
            .iter()
            .map(|fixture| fixture["scale_parameter"].as_integer().unwrap())
            .collect::<Vec<_>>(),
        [0, 64, 512, 2048]
    );

    let mut topology = None;
    for shift in support::WIDE_RATIONAL_SHIFTS {
        let scale = support::wide_rational_scale(shift)
            .exact_rational()
            .expect("fixture similarity is exact rational");
        assert_eq!(scale.numerator().bits(), u64::from(shift) + 1);
        assert_eq!(scale.denominator().bits(), u64::from(shift) + 1);
        assert_eq!(
            support::wide_rational_scale(shift).to_f64_lossy(),
            Some(1.0),
        );

        let case =
            support::wide_rational_overlapping_box_case(support::WIDE_RATIONAL_DIVISIONS, shift);
        assert_eq!(
            case.left.triangles.len() + case.right.triangles.len(),
            6_144
        );
        let current = [case.left.triangles.to_vec(), case.right.triangles.to_vec()];
        if let Some(topology) = &topology {
            assert_eq!(&current, topology);
        } else {
            topology = Some(current);
        }
        assert!(
            case.left
                .positions
                .iter()
                .chain(case.right.positions.iter())
                .flat_map(|point| [&point.x, &point.y, &point.z])
                .all(|coordinate| coordinate.exact_rational_ref().is_some())
        );

        let fixture = family
            .iter()
            .find(|fixture| fixture["scale_parameter"].as_integer() == Some(i64::from(shift)))
            .expect("every generated shift is manifested");
        assert_eq!(
            fixture["similarity_component_bits"].as_integer(),
            Some(i64::from(shift) + 1)
        );
        assert_eq!(fixture["input_triangles"].as_integer(), Some(6_144));
    }
}

#[test]
fn dense_coplanar_family_scales_mesh_work_at_fixed_coordinate_complexity() {
    let manifest = manifest();
    let family = manifest["fixture"]
        .as_array()
        .expect("fixture records")
        .iter()
        .filter(|fixture| {
            fixture.get("scaling_family").and_then(Value::as_str) == Some("dense_coplanar_boxes")
        })
        .collect::<Vec<_>>();
    assert_eq!(family.len(), 3);
    assert_eq!(
        family
            .iter()
            .map(|fixture| fixture["scale_parameter"].as_integer().unwrap())
            .collect::<Vec<_>>(),
        support::DENSE_COPLANAR_DIVISIONS.map(|divisions| divisions as i64)
    );
    assert!(
        family.iter().all(|fixture| {
            fixture["input_coordinate_denominator_bound"].as_integer() == Some(8)
        })
    );

    for divisions in support::DENSE_COPLANAR_DIVISIONS {
        let expected_triangles = 24 * divisions * divisions;
        let case = support::dense_coplanar_box_case(divisions);
        assert_eq!(
            case.left.triangles.len() + case.right.triangles.len(),
            expected_triangles
        );
        for mesh in [&case.left, &case.right] {
            let summary = support::summarize(mesh);
            assert!(summary.closed);
            assert!(summary.nondegenerate);
            assert_eq!(summary.volume, 64.0);
            assert!(
                mesh.positions
                    .iter()
                    .flatten()
                    .all(|coordinate| (coordinate * 8.0).fract() == 0.0)
            );
        }
        assert_eq!(
            family
                .iter()
                .find(|fixture| {
                    fixture["scale_parameter"].as_integer() == Some(divisions as i64)
                })
                .unwrap()["input_triangles"]
                .as_integer(),
            Some(expected_triangles as i64)
        );
    }
}

#[test]
fn clipped_voxel_torus_family_spans_medium_large_and_xl_exactly() {
    let manifest = manifest();
    let family = manifest["fixture"]
        .as_array()
        .expect("fixture records")
        .iter()
        .filter(|fixture| {
            fixture.get("scaling_family").and_then(Value::as_str) == Some("clipped_voxel_torus")
        })
        .collect::<Vec<_>>();
    assert_eq!(family.len(), 3);
    assert_eq!(
        family
            .iter()
            .map(|fixture| fixture["scale_parameter"].as_integer().unwrap())
            .collect::<Vec<_>>(),
        [9, 33, 65]
    );
    assert_eq!(
        family
            .iter()
            .map(|fixture| fixture["size"].as_str().unwrap())
            .collect::<Vec<_>>(),
        ["medium", "large", "xl"]
    );
    for (extent, expected_triangles) in [(9, 460), (33, 6_412), (65, 25_100)] {
        let case = support::clipped_voxel_torus_case(extent);
        assert_eq!(
            case.left.triangles.len() + case.right.triangles.len(),
            expected_triangles
        );
        assert_eq!(
            family
                .iter()
                .find(|fixture| fixture["scale_parameter"].as_integer() == Some(extent as i64))
                .unwrap()["input_triangles"]
                .as_integer(),
            Some(expected_triangles as i64)
        );
    }
}

#[test]
fn lower_dimensional_contact_fuzz_seeds_are_pinned_and_executable() {
    let manifest = manifest();
    let fixtures = manifest["fixture"].as_array().expect("fixture records");
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    for (id, relative, expected) in [
        (
            "edge_touching_boxes",
            "fuzz/seeds/boolean_box_oracle/edge_touching_boxes",
            b"ADDGGGBBBIIGBBBXXXXXXXXXXXXXXXX\n".as_slice(),
        ),
        (
            "vertex_touching_boxes",
            "fuzz/seeds/boolean_box_oracle/vertex_touching_boxes",
            b"ADDGGGBBBIIIBBBXXXXXXXXXXXXXXXX\n".as_slice(),
        ),
        (
            "face_tangent_containment_boxes",
            "fuzz/seeds/boolean_box_oracle/face_tangent_containment_boxes",
            b"ADDGGGDDDGHHBBBXXXXXXXXXXXXXXXX\n".as_slice(),
        ),
    ] {
        let fixture = fixtures
            .iter()
            .find(|fixture| fixture["id"].as_str() == Some(id))
            .unwrap_or_else(|| panic!("missing fixture {id}"));
        assert_eq!(fixture["fuzz_seed"].as_str(), Some(relative));
        assert_eq!(
            std::fs::read(root.join(relative)).unwrap_or_else(|error| {
                panic!("failed to read permanent fuzz seed {relative}: {error}")
            }),
            expected
        );
        assert_eq!(expected.len(), 32);
    }
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
