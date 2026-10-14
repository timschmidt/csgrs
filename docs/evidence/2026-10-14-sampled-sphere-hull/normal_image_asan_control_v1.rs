//! A deliberate error used only to check the isolated sanitizer configuration.
fn main() {
    let mut allocation = Box::new([0_u8; 1]);
    let pointer = std::hint::black_box(allocation.as_mut_ptr());
    let offset = std::hint::black_box(2_usize);
    // This binary is expected to fail under ASan. It is never linked into a crate.
    unsafe { pointer.add(offset).write_volatile(1) };
    std::hint::black_box(allocation);
    panic!("the deliberate out-of-bounds write was not detected by ASan");
}
