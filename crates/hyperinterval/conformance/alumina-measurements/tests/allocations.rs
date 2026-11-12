use alumina_interface_core::{BoundaryError, BoundedMeasurement, Millimetres};
use hyperreal::Rational;
use interval_measurement_conformance::sample;
#[path = "../../../../hyperlattice-interval/conformance/allocations.rs"]
mod allocations;

#[test]
fn cold_and_warm_allocations_match_in_fresh_processes() {
    const SIDE: &str = "MEASUREMENT_ALLOC_SIDE";
    const INDEX: &str = "MEASUREMENT_ALLOC_INDEX";
    const WARM: &str = "MEASUREMENT_ALLOC_WARM";
    if let Ok(side) = std::env::var(SIDE) {
        let index = std::env::var(INDEX).unwrap().parse::<usize>().unwrap();
        let (n, u) = sample(index);
        let run = |n: Rational, u: Rational| {
            if side == "original" {
                if u.is_negative() {
                    return Err(BoundaryError::NegativeUncertainty);
                }
                Ok((n.clone() - u.clone(), n + u))
            } else {
                assert_eq!(side, "public");
                BoundedMeasurement::<Millimetres>::from_nominal_uncertainty(n, u)
                    .map(|x| (x.lower().clone(), x.upper().clone()))
            }
        };
        if std::env::var(WARM).unwrap() == "true" {
            let _ = std::hint::black_box(run(n.clone(), u.clone()));
        }
        let (result, calls, bytes) = allocations::measure(|| run(n, u));
        println!("ALLOC={calls},{bytes}");
        if index == 7 {
            assert_eq!((calls, bytes), (0, 0));
        }
        match result {
            Ok(bounds) => println!("RESULT={}", serde_json::to_string(&bounds).unwrap()),
            Err(error) => println!("RESULT={error:?}"),
        }
        return;
    }
    // Each subprocess runs this executable's sole test. Input construction and
    // serialization are outside the counter; the full arithmetic output is kept.
    for index in 0..8 {
        for warm in [false, true] {
            let mut evidence = Vec::new();
            for side in ["original", "public"] {
                let child = std::process::Command::new(std::env::current_exe().unwrap())
                    .args([
                        "--exact",
                        "cold_and_warm_allocations_match_in_fresh_processes",
                        "--nocapture",
                    ])
                    .env(SIDE, side)
                    .env(INDEX, index.to_string())
                    .env(WARM, warm.to_string())
                    .output()
                    .unwrap();
                assert!(
                    child.status.success(),
                    "{} {}",
                    String::from_utf8_lossy(&child.stdout),
                    String::from_utf8_lossy(&child.stderr)
                );
                let output = String::from_utf8(child.stdout).unwrap();
                let lines: Vec<_> = output
                    .lines()
                    .filter(|line| line.starts_with("ALLOC=") || line.starts_with("RESULT="))
                    .map(str::to_owned)
                    .collect();
                assert_eq!(lines.len(), 2, "{output}");
                evidence.push(lines);
            }
            assert_eq!(evidence[0], evidence[1], "fixture {index}, warm={warm}");
            println!("fixture={index}, warm={warm}, {}", evidence[0][0]);
        }
    }
}
