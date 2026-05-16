#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

// Deep alternating island/lake stacks keep pressure on containment-depth classification and on
// clipping only the immediate holes that belong to each intersected shell.
fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    let mut a = support::nested_rect_shape(&mut reader, 6);
    let mut b = if reader.bool() {
        support::nested_rect_shape(&mut reader, 6)
    } else if reader.bool() {
        support::donut_shape(&mut reader)
    } else {
        support::rectangle_shape(&mut reader, 3)
    };

    support::maybe_transform(&mut a, &mut reader);
    support::maybe_transform(&mut b, &mut reader);
    support::assert_boolean_semantics_all_ops(&a, &b);
    support::assert_boolean_identities(&a);
    support::assert_boolean_identities(&b);
});
