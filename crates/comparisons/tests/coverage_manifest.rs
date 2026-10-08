use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

use exactcore_hyper_comparison::COMPARABLE_OPERATION_IDS;

const COMPARISON_HEADER: [&str; 9] = [
    "id",
    "status",
    "exactcore_api",
    "hyper_api",
    "hyper_crates",
    "oracle_symbols",
    "tests",
    "benchmarks",
    "notes",
];

const CRATE_HEADER: [&str; 5] = ["crate", "status", "direct_surface", "suite", "notes"];

fn project_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn read_tsv(path: &Path, expected_header: &[&str]) -> Vec<Vec<String>> {
    let source = fs::read_to_string(path)
        .unwrap_or_else(|error| panic!("could not read {}: {error}", path.display()));
    let mut lines = source.lines().filter(|line| !line.trim().is_empty());
    let header = lines.next().expect("TSV must have a header");
    let actual_header = header.split('\t').collect::<Vec<_>>();
    assert_eq!(
        actual_header,
        expected_header,
        "header in {}",
        path.display()
    );

    lines
        .enumerate()
        .map(|(index, line)| {
            let fields = line.split('\t').map(str::to_owned).collect::<Vec<_>>();
            assert_eq!(
                fields.len(),
                expected_header.len(),
                "{}:{} must have {} tab-separated fields",
                path.display(),
                index + 2,
                expected_header.len()
            );
            assert!(
                fields.iter().all(|field| !field.trim().is_empty()),
                "{}:{} has an empty field",
                path.display(),
                index + 2
            );
            fields
        })
        .collect()
}

fn cargo_package_name(manifest: &Path) -> String {
    let source = fs::read_to_string(manifest)
        .unwrap_or_else(|error| panic!("could not read {}: {error}", manifest.display()));
    let mut in_package = false;
    for line in source.lines() {
        let line = line.trim();
        if line == "[package]" {
            in_package = true;
            continue;
        }
        if in_package && line.starts_with('[') {
            break;
        }
        if in_package && line.starts_with("name") {
            let (_, value) = line
                .split_once('=')
                .unwrap_or_else(|| panic!("malformed package name in {}", manifest.display()));
            return value.trim().trim_matches('"').to_owned();
        }
    }
    panic!("no [package] name in {}", manifest.display());
}

fn sibling_hyper_packages() -> BTreeSet<String> {
    let root = project_root();
    let workspace = root
        .parent()
        .expect("comparison project must have a parent");
    fs::read_dir(workspace)
        .expect("read workspace")
        .filter_map(Result::ok)
        .filter(|entry| entry.file_name().to_string_lossy().starts_with("hyper"))
        .map(|entry| entry.path().join("Cargo.toml"))
        .filter(|manifest| manifest.is_file())
        .map(|manifest| cargo_package_name(&manifest))
        .filter(|name| name.starts_with("hyper"))
        .collect()
}

fn listed_paths(field: &str) -> impl Iterator<Item = &str> {
    field.split(';').map(str::trim).filter(|path| *path != "-")
}

fn source_tree(directory: &Path) -> String {
    let mut source = String::new();
    for entry in fs::read_dir(directory)
        .unwrap_or_else(|error| panic!("could not read {}: {error}", directory.display()))
    {
        let path = entry.expect("directory entry").path();
        if path.extension().and_then(|extension| extension.to_str()) == Some("rs") {
            source.push_str(
                &fs::read_to_string(&path)
                    .unwrap_or_else(|error| panic!("could not read {}: {error}", path.display())),
            );
            source.push('\n');
        }
    }
    source
}

fn c_oracle_symbols(header: &str) -> BTreeSet<String> {
    header
        .split(|character: char| !(character.is_ascii_alphanumeric() || character == '_'))
        .filter(|word| word.starts_with("ec_") && word.len() > 3)
        .map(str::to_owned)
        .collect()
}

fn benchmark_operation_ids(source: &str) -> BTreeSet<String> {
    const PREFIXES: [&str; 14] = [
        "rational.",
        "real.",
        "complex.",
        "vector2.",
        "vector3.",
        "vector4.",
        "matrix3.",
        "matrix4.",
        "geometry2.",
        "geometry3.",
        "polynomial.",
        "bivariate.",
        "triangulation.",
        "curve.",
    ];
    const EXTRA_PREFIXES: [&str; 2] = ["mesh.", "path."];

    let mut ids = BTreeSet::new();
    for literal in source.split('"').skip(1).step_by(2) {
        let generated = match literal {
            "geometry2.line_relation_{operation}" => Some(("geometry2.line_relation_", 6)),
            "geometry2.segment_relation_{operation}" => Some(("geometry2.segment_relation_", 3)),
            "geometry3.line_relation_{operation}" => Some(("geometry3.line_relation_", 4)),
            "geometry3.segment_relation_{operation}" => Some(("geometry3.segment_relation_", 3)),
            "geometry3.plane_relation_{operation}" => Some(("geometry3.plane_relation_", 7)),
            "geometry3.triangle_relation_{operation}" => Some(("geometry3.triangle_relation_", 6)),
            "polynomial.binary_{operation}" => Some(("polynomial.binary_", 4)),
            _ => None,
        };
        if let Some((prefix, maximum)) = generated {
            ids.extend((0..=maximum).map(|index| format!("{prefix}{index}")));
        } else if PREFIXES
            .iter()
            .chain(EXTRA_PREFIXES.iter())
            .any(|prefix| literal.starts_with(prefix))
            && !literal.contains('{')
        {
            ids.insert(literal.to_owned());
        }
    }
    ids
}

#[test]
fn retained_benchmarks_and_memory_catalog_have_identical_operation_ids() {
    let root = project_root();
    let benchmark_source = [
        "benches/comparable.rs",
        "benches/curves.rs",
        "benches/meshes.rs",
        "benches/paths.rs",
    ]
    .into_iter()
    .map(|path| fs::read_to_string(root.join(path)).expect("read benchmark source"))
    .collect::<Vec<_>>()
    .join("\n");
    let benchmark_ids = benchmark_operation_ids(&benchmark_source);
    let memory_ids = COMPARABLE_OPERATION_IDS
        .iter()
        .map(|id| (*id).to_owned())
        .collect::<BTreeSet<_>>();
    assert_eq!(
        memory_ids, benchmark_ids,
        "the memory sweep must cover every concrete retained benchmark operation"
    );
}

#[test]
fn every_hyper_crate_has_an_explicit_audit_disposition() {
    let root = project_root();
    let rows = read_tsv(&root.join("coverage/hyper-crates.tsv"), &CRATE_HEADER);
    let mut audited = BTreeMap::new();
    let valid_statuses = [
        "compared",
        "blocked",
        "comparison-pending",
        "no-direct-counterpart",
    ];

    for row in rows {
        let name = &row[0];
        let status = &row[1];
        assert!(
            valid_statuses.contains(&status.as_str()),
            "unknown audit status {status} for {name}"
        );
        assert!(
            audited.insert(name.clone(), status.clone()).is_none(),
            "duplicate crate-audit row for {name}"
        );

        if status == "compared" {
            let paths = listed_paths(&row[3]).collect::<Vec<_>>();
            assert!(
                paths.iter().any(|path| path.starts_with("tests/")),
                "compared crate {name} must name a test"
            );
            assert!(
                paths.iter().any(|path| path.starts_with("benches/")),
                "compared crate {name} must name a benchmark"
            );
            for path in paths {
                assert!(root.join(path).is_file(), "missing suite file {path}");
            }
        } else {
            assert_eq!(row[3], "-", "non-compared crate {name} has a suite");
            assert_ne!(row[4], "-", "non-compared crate {name} needs a reason");
        }
    }

    let discovered = sibling_hyper_packages();
    let listed = audited.keys().cloned().collect::<BTreeSet<_>>();
    assert_eq!(
        listed, discovered,
        "the crate audit must exactly match sibling hyper* Cargo packages"
    );
    assert_eq!(
        audited.get("hyperbrep").map(String::as_str),
        Some("comparison-pending")
    );
}

#[test]
fn every_comparable_row_names_tests_benchmarks_and_known_crates() {
    let root = project_root();
    let comparison_rows = read_tsv(
        &root.join("coverage/comparable-api.tsv"),
        &COMPARISON_HEADER,
    );
    let crate_rows = read_tsv(&root.join("coverage/hyper-crates.tsv"), &CRATE_HEADER);
    let known_crates = crate_rows
        .iter()
        .map(|row| row[0].as_str())
        .collect::<BTreeSet<_>>();
    let mut compared_crates = BTreeSet::new();
    let mut identifiers = BTreeSet::new();
    let valid_statuses = ["matched", "adapted", "divergent", "blocked"];

    for row in &comparison_rows {
        let identifier = &row[0];
        let status = &row[1];
        assert!(
            identifiers.insert(identifier.clone()),
            "duplicate comparable API id {identifier}"
        );
        assert!(
            valid_statuses.contains(&status.as_str()),
            "unknown comparison status {status} for {identifier}"
        );

        for crate_name in row[4].split(';') {
            assert!(
                known_crates.contains(crate_name),
                "comparison {identifier} names unknown crate {crate_name}"
            );
            compared_crates.insert(crate_name);
        }

        if status == "blocked" {
            assert_eq!(row[5], "-", "blocked row {identifier} invokes an oracle");
            assert_eq!(row[6], "-", "blocked row {identifier} names a test");
            assert_eq!(row[7], "-", "blocked row {identifier} names a benchmark");
            assert!(
                row[8].to_ascii_lowercase().contains("not executed"),
                "blocked row {identifier} must explain why it is not executed"
            );
            continue;
        }

        assert_ne!(row[5], "-", "comparison {identifier} lacks an oracle");
        assert_ne!(row[6], "-", "comparison {identifier} lacks a test");
        assert_ne!(row[7], "-", "comparison {identifier} lacks a benchmark");
        for path in listed_paths(&row[6]).chain(listed_paths(&row[7])) {
            assert!(
                root.join(path).is_file(),
                "comparison {identifier} names missing source {path}"
            );
        }
        if status == "divergent" {
            assert!(
                row[8].to_ascii_lowercase().contains("ignored"),
                "divergence {identifier} must point to an ignored regression"
            );
            assert!(
                listed_paths(&row[6]).any(|path| {
                    fs::read_to_string(root.join(path))
                        .expect("read divergence test")
                        .contains("#[ignore")
                }),
                "divergence {identifier} has no ignored test in its named sources"
            );
        }
    }

    let expected_compared = crate_rows
        .iter()
        .filter(|row| row[1] == "compared")
        .map(|row| row[0].as_str())
        .collect::<BTreeSet<_>>();
    assert_eq!(
        compared_crates, expected_compared,
        "each compared crate must own at least one comparable API row"
    );

    let expected_divergences = [
        "matrix-determinant",
        "line-relations-2d",
        "circle-line-segment-2d",
        "plane-relations-3d",
        "triangle-relations-3d",
        "constant-resultant",
        "polynomial-discriminant",
        "polynomial-root-count",
        "curve-degenerate-line",
        "mesh-convex-triangles",
        "path-contained-segment",
    ];
    for identifier in expected_divergences {
        assert!(
            comparison_rows
                .iter()
                .any(|row| row[0] == identifier && row[1] == "divergent"),
            "known divergence {identifier} disappeared from the manifest"
        );
    }
}

#[test]
fn every_native_oracle_entry_point_is_mapped_tested_and_benchmarked() {
    let root = project_root();
    let header = fs::read_to_string(root.join("cpp/exactcore_oracle.h"))
        .expect("read exactCore oracle header");
    let declared = c_oracle_symbols(&header);
    let rows = read_tsv(
        &root.join("coverage/comparable-api.tsv"),
        &COMPARISON_HEADER,
    );
    let mapped = rows
        .iter()
        .flat_map(|row| row[5].split(';'))
        .filter(|symbol| *symbol != "-")
        .map(str::to_owned)
        .collect::<BTreeSet<_>>();
    assert_eq!(
        mapped, declared,
        "oracle header and comparable API manifest must cover the same symbols"
    );

    let tests = source_tree(&root.join("tests"));
    let benches = source_tree(&root.join("benches"));
    for symbol in declared {
        let wrapper = match symbol.as_str() {
            "ec_bivariate_resultant" => "bivariate_resultant_eval",
            _ => symbol.strip_prefix("ec_").expect("ec_ prefix"),
        };
        assert!(
            tests.contains(wrapper),
            "native oracle {symbol} has no differential-test call through {wrapper}"
        );
        assert!(
            benches.contains(wrapper),
            "native oracle {symbol} has no paired benchmark call through {wrapper}"
        );
    }
}

#[test]
fn exhaustive_report_mentions_every_manifest_row_and_crate() {
    let root = project_root();
    let report = fs::read_to_string(root.join("COMPARISON_REPORT.md"))
        .expect("read exhaustive comparison report");
    let comparison_rows = read_tsv(
        &root.join("coverage/comparable-api.tsv"),
        &COMPARISON_HEADER,
    );
    let crate_rows = read_tsv(&root.join("coverage/hyper-crates.tsv"), &CRATE_HEADER);

    for row in &comparison_rows {
        let identifier = &row[0];
        let status = &row[1];
        let table_prefix = format!("| `{identifier}` | {status} |");
        assert!(
            report.lines().any(|line| line.starts_with(&table_prefix)),
            "report lacks the {status} comparison row {identifier}"
        );
    }

    for row in &crate_rows {
        let name = &row[0];
        let status = &row[1];
        let displayed_status = status.replace('-', " ");
        let table_prefix = format!("| `{name}` |");
        let line = report
            .lines()
            .find(|line| line.starts_with(&table_prefix))
            .unwrap_or_else(|| panic!("report lacks the crate disposition for {name}"));
        assert!(
            line.contains(&displayed_status),
            "report crate row for {name} does not preserve status {status}"
        );
    }

    let status_counts = comparison_rows
        .iter()
        .fold(BTreeMap::new(), |mut counts, row| {
            *counts.entry(row[1].as_str()).or_insert(0usize) += 1;
            counts
        });
    for (status, count) in status_counts {
        assert!(
            report.contains(&format!("{count}")) && report.contains(status),
            "report does not state the manifest count {count} for {status} rows"
        );
    }
}
