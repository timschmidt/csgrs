#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

// A concentrated corpus of the cases that have historically exposed shape-boolean bugs: deep
// island/lake nesting, dense hole grids, arc rings, sawtooth intersections, near-coincident
// shared edges, and the complex multipolyline scene from the UI demo.
fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    let (a, b) = support::adversarial_corpus_pair(&mut reader);
    if reader.bool() {
        support::assert_boolean_semantics_all_ops(&a, &b);
    } else {
        support::assert_boolean_semantics(&a, &b, support::boolean_op(&mut reader));
    }
    support::assert_boolean_identities(&a);
    support::assert_boolean_identities(&b);
});
