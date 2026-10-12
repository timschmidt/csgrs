use std::hint::black_box;

fn main() {
    let mut state = 0x9e37_79b9_7f4a_7c15_u64;
    for _ in 0..50_000_000 {
        state = black_box(state.wrapping_mul(6364136223846793005).wrapping_add(1));
    }
    println!("{state}");
}
