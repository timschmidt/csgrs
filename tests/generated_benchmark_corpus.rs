//! Import validity and deterministic bytes for two generated benchmark OBJ fixtures.
//!
//! Generation rewrites only changed bytes in the selected target directory.
//! These checks do not compare an independent exact set oracle or certify source
//! fidelity through general OBJ projection; the fixtures use small integer vertices.

/// Shared voxel construction, fixed-count validation and on-disk installation.
#[path = "../benchmarks/support/generated_corpus.rs"]
mod generated_corpus;

use std::{fs::File, io::BufReader};

/// Imports both generated fixtures, checks native closed-manifold geometry and byte stability.
///
/// Each generator runs twice; the second file read must match the first exactly.
/// This is same-process regeneration, not a fresh-process replay or concurrent
/// writer test. Generated OBJ files remain in the shared target fixture directory.
#[test]
fn generated_competitive_corpus_is_closed_and_deterministic() {
    for generate in [
        generated_corpus::concave_path as fn() -> std::path::PathBuf,
        generated_corpus::sierpinski_foam_path,
    ] {
        let path = generate();
        let first = std::fs::read(&path).expect("generated corpus bytes");
        let mesh = csgrs::io::obj::from_obj(BufReader::new(
            File::open(&path).expect("generated corpus OBJ"),
        ))
        .expect("generated corpus imports");
        assert!(mesh.is_closed_manifold());
        let second = std::fs::read(generate()).expect("stable generated corpus bytes");
        assert_eq!(first, second);
    }
}
