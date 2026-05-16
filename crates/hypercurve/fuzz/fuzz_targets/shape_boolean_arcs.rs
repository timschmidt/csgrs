#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

// Arc-bearing loops stress bulge handling, tangent contacts, and SVG/debug reductions that would
// be invisible in a rectangle-only corpus.
fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    let a = support::arc_shape(&mut reader);
    let b = support::arc_shape(&mut reader);
    let op = support::boolean_op(&mut reader);
    support::assert_boolean_semantics(&a, &b, op);
});
