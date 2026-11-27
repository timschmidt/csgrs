/// Run 64 consecutive wrapping u32 sample indices through all five actions using the imported provider.
///
/// The optional first CLI argument is a u32 seed, defaulting to zero. Malformed
/// or out-of-range seeds panic; later arguments are ignored. Input construction,
/// transform execution and digest updates happen here without timing or allocation
/// instrumentation. All results are unwrapped and folded into one FNV-1a digest
/// starting at its conventional offset basis, printed as sixteen lowercase hex digits.
/// This shared body supports comparable linked programs; a digest alone does not
/// certify full output equality, and no timeout or firmware-size bound is supplied.
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
