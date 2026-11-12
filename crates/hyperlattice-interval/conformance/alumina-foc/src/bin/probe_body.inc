fn main() {
    use interval_foc_conformance::samples::{ACTIONS, sample};
    let seed = std::env::args()
        .nth(1)
        .map(|s| s.parse::<u32>().unwrap())
        .unwrap_or(0);
    let mut hash = 0xcbf29ce484222325;
    for index in 0..64 {
        for action in ACTIONS {
            hash = provider(
                std::hint::black_box(action),
                std::hint::black_box(sample(seed.wrapping_add(index))),
            )
            .unwrap()
            .digest(hash);
        }
    }
    println!("{hash:016x}");
}
