#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

// Sweep a single vertex around the full scene, rebuilding shape indexes after each movement. This
// isolates lower-level boolean robustness from the UI's per-frame index update behavior.
fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    support::assert_vertex_drag_boolean_sequence(&mut reader, true);
});
