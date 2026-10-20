//! Cost of the completed exact inverse → ECEF → local-frame workflow.
use std::{hint::black_box, time::Instant};

#[path = "../dev/inverse_isometric.rs"]
mod inverse_isometric;
#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--test");
    let iterations = if smoke { 1 } else { 20 };
    let trials = if smoke { 1 } else { 5 };
    let input = [0, 7, 5, 1, 3, 0];
    inverse_isometric::exercise(input).unwrap();
    println!("operation,trial,iterations,elapsed_ns,completed,certainty");
    for trial in 0..trials {
        let start = Instant::now();
        for _ in 0..iterations {
            black_box(inverse_isometric::exercise(input).unwrap());
        }
        println!(
            "inverse_isometric_ecef_local,{trial},{iterations},{},{iterations},Exact",
            start.elapsed().as_nanos()
        );
    }
}
