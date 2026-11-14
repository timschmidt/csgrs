//! Repeated complete native surface-range queries with preparation included.
#[path = "support/surface_ranges.rs"]
mod workload;
use std::{hint::black_box, time::Instant};
use workload::{arithmetic, compare, complete, graph, source, validate};

fn main() {
    let iterations: u32 = std::env::var("SURFACE_RANGE_ITERATIONS")
        .map(|s| s.parse().unwrap())
        .unwrap_or(16);
    let samples: u32 = std::env::var("SURFACE_RANGE_SAMPLES")
        .map(|s| s.parse().unwrap())
        .unwrap_or(30);
    assert!(iterations > 0 && samples > 0);
    for (case, name) in ["polynomial", "rational", "degree8x3"]
        .into_iter()
        .enumerate()
    {
        let source = source(case);
        let expected = complete(&source, false, &mut graph(), &mut arithmetic());
        let shared = complete(&source, true, &mut graph(), &mut arithmetic());
        validate(case, &expected);
        validate(case, &shared);
        compare(&expected, &shared);
        drop(shared);
        for sample in 0..samples + 2 {
            for shared in if sample % 2 == 0 {
                [false, true]
            } else {
                [true, false]
            } {
                let start = Instant::now();
                for _ in 0..iterations {
                    let reports =
                        complete(black_box(&source), shared, &mut graph(), &mut arithmetic());
                    for (actual, expected) in reports.iter().zip(&expected) {
                        assert_eq!(actual.evidence().values(), expected.evidence().values());
                    }
                    drop(black_box(reports));
                }
                if sample >= 2 {
                    println!(
                        "{name}/{} sample={} iterations={iterations} total_ns={}",
                        if shared { "shared" } else { "independent" },
                        sample - 2,
                        start.elapsed().as_nanos()
                    );
                }
            }
        }
    }
}
