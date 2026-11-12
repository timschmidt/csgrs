//! Proposal-only completed-workload cost probe. No certification claim.
use hyperinterval::{F32ProposalInterval, F64ProposalInterval};
use std::{hint::black_box, time::Instant};

fn main() {
    eprintln!(
        "layout_bytes,f32={},f64={}",
        size_of::<F32ProposalInterval>(),
        size_of::<F64ProposalInterval>()
    );
    println!("width,sample,elapsed_ns,completed_steps,checksum");
    macro_rules! measure {
        ($interval:ident, $width:literal) => {
            let initial = $interval::new(0.75, 1.25).unwrap();
            let drift = $interval::new(-0.125, 0.25).unwrap();
            for sample in 0..30 {
                let started = Instant::now();
                let mut checksum = 0u64;
                for _ in 0..16384 {
                    let result = black_box(initial)
                        .add(&black_box(drift))
                        .unwrap()
                        .affine(black_box(0.5), black_box(0.25))
                        .unwrap();
                    let (a, b) = black_box(result).estimates().unwrap();
                    checksum =
                        checksum.wrapping_add(u64::from(a.to_bits()) ^ u64::from(b.to_bits()));
                }
                println!(
                    "{},{sample},{},16384,{checksum}",
                    $width,
                    started.elapsed().as_nanos()
                );
            }
        };
    }
    measure!(F32ProposalInterval, 32);
    measure!(F64ProposalInterval, 64);
}
