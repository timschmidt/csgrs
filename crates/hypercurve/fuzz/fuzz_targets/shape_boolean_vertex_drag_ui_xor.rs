#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

// XOR has a distinct composition path and tends to expose different transient topology while a UI
// vertex is dragged around the multipolyline boolean demo. Keep this target XOR-only so coverage
// and corpus growth concentrate on that mode instead of being diluted across all booleans.
fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    support::assert_vertex_drag_xor_sequence(&mut reader, false);
});
