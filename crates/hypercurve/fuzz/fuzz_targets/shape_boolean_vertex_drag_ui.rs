#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

// Mimic the interactive UI path: dragging updates the edited loop and its child spatial index,
// while the enclosing shape may still carry the previous top-level index during the frame. This
// targets flashing/crashing seen when a vertex is swept around complex boolean inputs.
fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    support::assert_vertex_drag_boolean_sequence(&mut reader, false);
});
