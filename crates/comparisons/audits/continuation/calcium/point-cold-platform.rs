include!("point-cold-common.rs");

use std::sync::Mutex;
static OUTPUT: Mutex<Vec<u8>> = Mutex::new(Vec::new());

#[unsafe(no_mangle)]
pub extern "C" fn cold_collect(
    which: usize,
    policy: usize,
    history: usize,
    lifecycle: usize,
) -> usize {
    let mut output = OUTPUT.lock().unwrap();
    *output = collect_cold_sequence(which, policy, history, lifecycle);
    output.len()
}

#[unsafe(no_mangle)]
pub extern "C" fn cold_output_ptr() -> *const u8 {
    OUTPUT.lock().unwrap().as_ptr()
}
