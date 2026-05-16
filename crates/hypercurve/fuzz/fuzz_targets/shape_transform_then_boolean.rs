#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

// Transforming before booleaning checks that translated, scaled, and rotated shapes keep their
// loop indexes and sampled set semantics coherent.
fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    let mut a = support::rectangle_shape(&mut reader, 3);
    let mut b = if reader.bool() {
        support::polygon_shape(&mut reader)
    } else {
        support::arc_shape(&mut reader)
    };

    support::maybe_transform(&mut a, &mut reader);
    support::maybe_transform(&mut b, &mut reader);

    let op = support::boolean_op(&mut reader);
    support::assert_boolean_semantics(&a, &b, op);
});
