#[path = "../benchmarks/support/generated_corpus.rs"]
mod generated_corpus;

use std::{fs::File, io::BufReader};

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
