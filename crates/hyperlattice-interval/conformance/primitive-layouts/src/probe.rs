use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure, FixedInterval};
use interval_layout_conformance::Soa;
use interval_layout_conformance::workload::{Fixture, inputs, native_chain, soa_chain};
use std::fmt::Debug;

fn family<I: Fixture + Debug, const N: usize, const SOA: bool>(seed: u32) {
    let (a, b) = inputs::<I, N>(seed);
    let result = if SOA {
        soa_chain(&Soa::from_vector(&a), &Soa::from_vector(&b))
    } else {
        native_chain(&a, &b)
    };
    println!("{} {N} {result:?}", core::any::type_name::<I>());
}

pub fn run<const SOA: bool>() {
    let seed = std::env::args().nth(1).unwrap().parse().unwrap();
    family::<FixedInterval<i32, 8>, 4, SOA>(seed);
    family::<FixedInterval<i32, 8>, 64, SOA>(seed);
    family::<FixedInterval<i64, 32>, 4, SOA>(seed);
    family::<FixedInterval<i64, 32>, 64, SOA>(seed);
    family::<F32Enclosure, 4, SOA>(seed);
    family::<F32Enclosure, 64, SOA>(seed);
    family::<F64Enclosure, 4, SOA>(seed);
    family::<F64Enclosure, 64, SOA>(seed);
}
