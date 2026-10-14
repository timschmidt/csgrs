//! Standalone sanitizer environment control; links no repository crates.
fn allocate_and_free() {
    let bytes = std::hint::black_box(vec![7_u8; 1024]);
    assert_eq!(std::hint::black_box(bytes.len()), 1024);
    drop(bytes);
}

#[inline(never)]
fn deliberate_leak() {
    let bytes = Box::new([7_u8; 4096]);
    std::hint::black_box(bytes.as_ptr());
    std::mem::forget(bytes);
}

fn main() {
    let mode = std::env::args().nth(1).expect("single, threads or leak");
    match mode.as_str() {
        "single" => allocate_and_free(),
        "threads" => {
            let first = std::thread::spawn(allocate_and_free);
            let second = std::thread::spawn(allocate_and_free);
            first.join().expect("first worker");
            second.join().expect("second worker");
        },
        "leak" => deliberate_leak(),
        _ => panic!("unknown control mode"),
    }
    println!("lsan_control={mode} completed");
}
