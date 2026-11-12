use alumina_interface_core::{BoundedMeasurement, Millimetres};
use interval_measurement_conformance::sample;
use std::{hint::black_box, time::Instant};
#[path = "../../../../../hyperlattice-interval/conformance/allocations.rs"]
mod allocations;

fn main() {
    println!("fixture,sample,completed,elapsed_ns,allocation_calls,allocated_bytes");
    for index in 0..8 {
        let (n, u) = sample(index);
        for _ in 0..32 {
            let _ = black_box(BoundedMeasurement::<Millimetres>::from_nominal_uncertainty(
                n.clone(),
                u.clone(),
            ));
        }
        for iteration in 0..20 {
            let start = Instant::now();
            let (_, calls, bytes) = allocations::measure(|| {
                for _ in 0..512 {
                    let result = BoundedMeasurement::<Millimetres>::from_nominal_uncertainty(
                        black_box(n.clone()),
                        black_box(u.clone()),
                    );
                    let _ = black_box(result);
                }
            });
            println!(
                "{index},{iteration},512,{},{calls},{bytes}",
                start.elapsed().as_nanos()
            );
        }
    }
}
