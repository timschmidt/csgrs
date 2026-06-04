//! Import boundary tests for the upstream Manifold test suite snapshot.
//!
//! The C++ gtest sources copied under `tests/upstream/manifold` are not built
//! by Cargo, but they still need to be a coherent, replayable test artifact.
//! These tests make the imported suite executable at the Rust boundary by
//! checking inventory, license provenance, referenced OBJ fixtures, and polygon
//! corpus syntax.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

const MANIFOLD_ROOT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/upstream/manifold");
const UPSTREAM_COMMIT: &str = "beb681b08048b56ec282e9097d74547d1bff53ee";
const EXPECTED_GTEST_COUNT: usize = 323;

const TEST_SOURCES: &[&str] = &[
    "boolean_complex_test.cpp",
    "boolean_test.cpp",
    "context_test.cpp",
    "cross_section_test.cpp",
    "hull_test.cpp",
    "manifold_fuzz.cpp",
    "manifold_test.cpp",
    "manifoldc_test.cpp",
    "measurement_test.cpp",
    "polygon_fuzz.cpp",
    "polygon_test.cpp",
    "properties_test.cpp",
    "samples_test.cpp",
    "sdf_test.cpp",
    "smooth_test.cpp",
    "test_main.cpp",
];

const OBJ_FIXTURES: &[&str] = &[
    "Cray_left.obj",
    "Cray_right.obj",
    "Generic_Twin_7081.1.t0_left.obj",
    "Generic_Twin_7081.1.t0_right.obj",
    "Generic_Twin_7863.1.t0_left.obj",
    "Generic_Twin_7863.1.t0_right.obj",
    "Havocglass8_left.obj",
    "Havocglass8_right.obj",
    "Offset1.obj",
    "Offset2.obj",
    "Offset3.obj",
    "Offset4.obj",
    "hull-body.obj",
    "hull-mask.obj",
    "openscad-nonmanifold-crash.obj",
    "self_intersectA.obj",
    "self_intersectB.obj",
];

const POLYGON_CORPORA: &[&str] = &[
    "polygon_corpus.txt",
    "sponge.txt",
    "zebra.txt",
    "zebra3.txt",
];

#[test]
fn upstream_manifold_test_suite_inventory_is_complete() {
    let root = Path::new(MANIFOLD_ROOT);
    assert!(
        root.join("LICENSE-APACHE-2.0").is_file(),
        "Apache-2.0 license must travel with imported Manifold tests"
    );
    let readme = fs::read_to_string(root.join("README.md")).expect("import README");
    assert!(
        readme.contains(UPSTREAM_COMMIT),
        "README must record the imported upstream commit"
    );

    let cmake = fs::read_to_string(root.join("test/CMakeLists.txt")).expect("Manifold CMake tests");
    let mut gtest_count = 0usize;
    for source in TEST_SOURCES {
        assert!(
            cmake.contains(source),
            "Manifold CMake test inventory lost {source}"
        );
        let path = root.join("test").join(source);
        assert!(
            path.is_file(),
            "missing imported Manifold test source {source}"
        );
        let text = fs::read_to_string(&path).expect("Manifold test source");
        gtest_count += count_gtest_macros(&text);
    }
    assert_eq!(
        gtest_count, EXPECTED_GTEST_COUNT,
        "unexpected Manifold gtest inventory size"
    );

    assert!(root.join("samples/include/samples.h").is_file());
    assert!(root.join("samples/src/gyroid_module.cpp").is_file());
    assert!(root.join("samples/src/menger_sponge.cpp").is_file());
}

#[test]
fn upstream_manifold_obj_fixtures_are_referenced_and_parseable() {
    let root = Path::new(MANIFOLD_ROOT);
    let referenced = referenced_obj_fixtures(root);
    for fixture in OBJ_FIXTURES {
        assert!(
            referenced.contains(*fixture),
            "OBJ fixture {fixture} is no longer referenced by upstream tests"
        );
    }

    let mut total_vertices = 0usize;
    let mut total_faces = 0usize;
    let mut total_triangulated_faces = 0usize;
    for fixture in OBJ_FIXTURES {
        let summary = parse_obj_summary(&root.join("test/models").join(fixture));
        assert!(summary.vertices > 0, "{fixture} has no OBJ vertices");
        assert!(summary.faces > 0, "{fixture} has no OBJ faces");
        assert_eq!(
            summary.non_tri_faces, 0,
            "{fixture} should remain a triangle-only Manifold fixture"
        );
        total_vertices += summary.vertices;
        total_faces += summary.faces;
        total_triangulated_faces += summary.triangulated_faces;
    }
    assert!(total_vertices > 10_000);
    assert_eq!(total_faces, total_triangulated_faces);
}

#[test]
fn upstream_manifold_polygon_corpora_are_structurally_readable() {
    let root = Path::new(MANIFOLD_ROOT).join("test/polygons");
    let mut total_cases = 0usize;
    let mut total_points = 0usize;
    for corpus in POLYGON_CORPORA {
        let summary = parse_polygon_corpus(&root.join(corpus));
        assert!(summary.cases > 0, "{corpus} has no polygon cases");
        assert!(summary.polygons >= summary.cases);
        assert!(summary.points >= summary.polygons * 3);
        total_cases += summary.cases;
        total_points += summary.points;
    }
    assert_eq!(total_cases, 109);
    assert!(total_points > 100_000);
}

fn count_gtest_macros(text: &str) -> usize {
    ["TEST(", "TEST_F(", "TEST_P("]
        .into_iter()
        .map(|needle| count_macro_invocations(text, needle))
        .sum()
}

fn count_macro_invocations(text: &str, needle: &str) -> usize {
    let mut count = 0usize;
    let mut rest = text;
    while let Some(offset) = rest.find(needle) {
        let absolute = text.len() - rest.len() + offset;
        let starts_identifier = absolute > 0
            && text[..absolute]
                .chars()
                .next_back()
                .is_some_and(|ch| ch == '_' || ch.is_ascii_alphanumeric());
        if !starts_identifier {
            count += 1;
        }
        rest = &rest[offset + needle.len()..];
    }
    count
}

fn referenced_obj_fixtures(root: &Path) -> BTreeSet<String> {
    let mut referenced = BTreeSet::new();
    for source in TEST_SOURCES {
        let path = root.join("test").join(source);
        let Ok(text) = fs::read_to_string(path) else {
            continue;
        };
        collect_string_arguments(&text, "ReadTestOBJ(\"", &mut referenced);
        collect_string_arguments(&text, "ReadTestMeshGL64OBJ(\"", &mut referenced);
    }
    referenced
}

fn collect_string_arguments(text: &str, marker: &str, out: &mut BTreeSet<String>) {
    let mut rest = text;
    while let Some(start) = rest.find(marker) {
        let after_marker = &rest[start + marker.len()..];
        let Some(end) = after_marker.find('"') else {
            break;
        };
        out.insert(after_marker[..end].to_owned());
        rest = &after_marker[end + 1..];
    }
}

#[derive(Debug, Default)]
struct ObjSummary {
    vertices: usize,
    faces: usize,
    triangulated_faces: usize,
    non_tri_faces: usize,
}

fn parse_obj_summary(path: &Path) -> ObjSummary {
    let text = fs::read_to_string(path)
        .unwrap_or_else(|error| panic!("failed to read OBJ fixture {}: {error}", path.display()));
    let mut summary = ObjSummary::default();
    for (line_no, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(rest) = line.strip_prefix("v ") {
            let coords: Vec<f64> = rest
                .split_whitespace()
                .map(|coord| {
                    coord.parse::<f64>().unwrap_or_else(|error| {
                        panic!(
                            "invalid OBJ vertex coordinate in {} at line {}: {error}",
                            path.display(),
                            line_no + 1
                        )
                    })
                })
                .collect();
            assert!(
                coords.len() >= 3 && coords[..3].iter().all(|coord| coord.is_finite()),
                "invalid OBJ vertex in {} at line {}",
                path.display(),
                line_no + 1
            );
            summary.vertices += 1;
        } else if let Some(rest) = line.strip_prefix("f ") {
            let indices: Vec<usize> = rest
                .split_whitespace()
                .map(|token| parse_obj_vertex_index(token, summary.vertices, path, line_no + 1))
                .collect();
            assert!(
                indices.len() >= 3,
                "OBJ face in {} at line {} has fewer than three vertices",
                path.display(),
                line_no + 1
            );
            if indices.len() != 3 {
                summary.non_tri_faces += 1;
            }
            summary.faces += 1;
            summary.triangulated_faces += indices.len() - 2;
        }
    }
    summary
}

fn parse_obj_vertex_index(token: &str, vertices: usize, path: &Path, line_no: usize) -> usize {
    let Some(raw_index) = token.split('/').next().filter(|value| !value.is_empty()) else {
        panic!(
            "missing OBJ face vertex index in {} at line {}",
            path.display(),
            line_no
        );
    };
    let index = raw_index.parse::<isize>().unwrap_or_else(|error| {
        panic!(
            "invalid OBJ face vertex index in {} at line {}: {error}",
            path.display(),
            line_no
        )
    });
    assert_ne!(
        index,
        0,
        "OBJ indices are one-based in {} at line {}",
        path.display(),
        line_no
    );
    let resolved = if index > 0 {
        index - 1
    } else {
        vertices as isize + index
    };
    assert!(
        resolved >= 0 && (resolved as usize) < vertices,
        "OBJ face references vertex {index} outside 1..={} in {} at line {}",
        vertices,
        path.display(),
        line_no
    );
    resolved as usize
}

#[derive(Debug, Default)]
struct PolygonCorpusSummary {
    cases: usize,
    polygons: usize,
    points: usize,
}

fn parse_polygon_corpus(path: &PathBuf) -> PolygonCorpusSummary {
    let text = fs::read_to_string(path).unwrap_or_else(|error| {
        panic!(
            "failed to read Manifold polygon corpus {}: {error}",
            path.display()
        )
    });
    let mut tokens = text.split_whitespace();
    let mut summary = PolygonCorpusSummary::default();
    while let Some(name) = tokens.next() {
        let expected = parse_next::<usize>(&mut tokens, path, name, "expected triangle count");
        let epsilon = parse_next::<f64>(&mut tokens, path, name, "epsilon");
        let polygons = parse_next::<usize>(&mut tokens, path, name, "polygon count");
        assert!(
            expected > 0,
            "polygon case {name} has no expected triangles"
        );
        assert!(
            epsilon.is_finite(),
            "polygon case {name} has non-finite epsilon"
        );
        assert!(polygons > 0, "polygon case {name} has no polygons");
        summary.cases += 1;
        summary.polygons += polygons;
        for _ in 0..polygons {
            let points = parse_next::<usize>(&mut tokens, path, name, "point count");
            assert!(points >= 3, "polygon case {name} has a degenerate ring");
            summary.points += points;
            for _ in 0..points {
                let x = parse_next::<f64>(&mut tokens, path, name, "x coordinate");
                let y = parse_next::<f64>(&mut tokens, path, name, "y coordinate");
                assert!(
                    x.is_finite() && y.is_finite(),
                    "polygon case {name} has non-finite coordinate"
                );
            }
        }
    }
    summary
}

fn parse_next<T>(
    tokens: &mut std::str::SplitWhitespace<'_>,
    path: &Path,
    case_name: &str,
    field: &str,
) -> T
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    let token = tokens.next().unwrap_or_else(|| {
        panic!(
            "missing {field} for polygon case {case_name} in {}",
            path.display()
        )
    });
    token.parse::<T>().unwrap_or_else(|error| {
        panic!(
            "invalid {field} for polygon case {case_name} in {}: {error}",
            path.display()
        )
    })
}
