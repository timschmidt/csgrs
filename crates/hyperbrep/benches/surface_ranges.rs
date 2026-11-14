//! Repeated complete native surface-range queries with preparation included.
#[path = "support/surface_ranges.rs"]
mod workload;
use std::{hint::black_box, time::Instant};
use workload::{arithmetic, compare, complete, graph, source, validate};

fn main() {
    if let Ok(case) = std::env::var("SURFACE_RANGE_COLD_CASE") {
        let case: usize = case.parse().unwrap();
        let shared = match std::env::var("SURFACE_RANGE_COLD_SHARED").unwrap().as_str() {
            "0" => false,
            "1" => true,
            _ => panic!("cold mode must be 0 or 1"),
        };
        let source = source(case);
        let start = Instant::now();
        let reports = complete(black_box(&source), shared, &mut graph(), &mut arithmetic());
        let query_elapsed = start.elapsed();
        // Independent oracle cost is excluded in both cold modes. Destruction
        // remains measured; the two spans and intervening oracle are explicit.
        validate(case, &reports);
        let drop_start = Instant::now();
        drop(black_box(reports));
        let destruction_elapsed = drop_start.elapsed();
        println!(
            "cold case={case} shared={shared} query_ns={} destruction_ns={} total_ns={}",
            query_elapsed.as_nanos(),
            destruction_elapsed.as_nanos(),
            (query_elapsed + destruction_elapsed).as_nanos()
        );
        return;
    }
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
