#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

// Concentrates mutation on mathematical singularities: point and edge contacts, epsilon-width
// overlaps, hole tangencies, collinear notch crossings, and large-coordinate precision loss.
fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    let (a, b) = support::singularity_corpus_pair(&mut reader);
    if reader.bool() {
        support::assert_boolean_semantics_all_ops(&a, &b);
    } else {
        support::assert_boolean_semantics(&a, &b, support::boolean_op(&mut reader));
    }
});
