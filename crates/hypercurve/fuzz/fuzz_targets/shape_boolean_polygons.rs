#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

// Polygon cases provide a straight-segment middle ground between exact rectangle oracles and
// arc-heavy cases, while still producing varied intersection topologies.
fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    let a = support::polygon_shape(&mut reader);
    let b = support::polygon_shape(&mut reader);
    let op = support::boolean_op(&mut reader);
    support::assert_boolean_semantics(&a, &b, op);
});
