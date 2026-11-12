use alumina_foc::FocError;
use interval_foc_conformance::samples::{self, ACTIONS, Action, Output, Sample};
use std::{hint::black_box, time::Instant};
#[path = "../../../allocations.rs"]
mod allocations;

fn main() {
    let inputs = core::array::from_fn::<_, 64, _>(|i| samples::sample(i as u32));
    for action in ACTIONS {
        for &input in &inputs {
            assert_eq!(
                samples::shared(action, input).unwrap(),
                samples::native(action, input).unwrap()
            );
        }
    }
    println!(
        "implementation,action,sample,completed_operations,elapsed_ns,allocation_calls,allocated_bytes,checksum"
    );
    type Provider = fn(Action, Sample) -> Result<Output, FocError>;
    for action in ACTIONS {
        for (name, provider) in [
            ("native", samples::native as Provider),
            ("shared", samples::shared as Provider),
        ] {
            for &input in &inputs {
                black_box(provider(black_box(action), black_box(input)).unwrap());
            }
            for sample in 0..20 {
                let mut hash = 0xcbf29ce484222325;
                let start = Instant::now();
                let (_, calls, bytes) = allocations::measure(|| {
                    for _ in 0..8 {
                        for &input in &inputs {
                            hash =
                                black_box(provider(black_box(action), black_box(input)).unwrap())
                                    .digest(hash);
                        }
                    }
                });
                let elapsed = start.elapsed().as_nanos();
                assert_eq!((calls, bytes), (0, 0));
                println!("{name},{action:?},{sample},512,{elapsed},{calls},{bytes},{hash:016x}");
            }
        }
    }
}
