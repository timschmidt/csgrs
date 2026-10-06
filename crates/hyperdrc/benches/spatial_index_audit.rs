use std::hint::black_box;
use std::time::Instant;

use hyperdrc::checks::drill_spacing;
use hyperdrc::kicad::DrillFeature;
use hyperdrc::scalar::scalar;

fn main() {
    let mut drills = (0..10_000)
        .map(|index| {
            let x = (index % 100) as i64 * 10;
            let y = (index / 100) as i64 * 10;
            drill(x, y)
        })
        .collect::<Vec<_>>();

    // Retain a small nearby cluster so the benchmark covers both empty and
    // populated candidate buckets without making exact arithmetic dominate.
    drills.extend([
        drill(2_000, 2_000),
        drill(2_000, 2_001),
        drill(2_001, 2_000),
    ]);

    let clearance = scalar("0.2");
    let iterations = if cfg!(debug_assertions) { 1 } else { 200 };
    let started = Instant::now();
    let mut finding_count = 0_usize;
    for _ in 0..iterations {
        finding_count += black_box(drill_spacing(
            black_box(&drills),
            black_box(&[]),
            black_box(&clearance),
        ))
        .len();
    }

    println!(
        "drill_spacing_10k iterations={iterations} findings={finding_count} ms={:.3}",
        started.elapsed().as_secs_f64() * 1_000.0
    );
}

fn drill(x_millimeters: i64, y_millimeters: i64) -> DrillFeature {
    DrillFeature {
        location: [
            scalar(&x_millimeters.to_string()),
            scalar(&y_millimeters.to_string()),
        ],
        diameter: scalar("0.4"),
        net: None,
        plated: true,
    }
}
