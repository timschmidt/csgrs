//! Shared command-line fixture runner for the two layout-specific executables.
//!
//! Each line prints the scalar type, dimension and complete Debug Result. These
//! diagnostics are not canonical serialization or a timing/qualification receipt.
//! Both programs use the same scalar kernels and bounded workload definitions.
use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure, FixedInterval};
use interval_layout_conformance::Soa;
use interval_layout_conformance::workload::{Fixture, inputs, native_chain, soa_chain};
use std::fmt::Debug;

/// Build one fixture and print its chain result using the compile-time layout choice.
/// SOA imports both inputs into separate arrays before the chain; false uses
/// native Cartesian storage. Construction is unwrapped by inputs, but a chain
/// error is printed as Err without making the process fail solely for that result.
fn family<I: Fixture + Debug, const N: usize, const SOA: bool>(seed: u32) {
    let (a, b) = inputs::<I, N>(seed);
    let result = if SOA {
        soa_chain(&Soa::from_vector(&a), &Soa::from_vector(&b))
    } else {
        native_chain(&a, &b)
    };
    println!("{} {N} {result:?}", core::any::type_name::<I>());
}

/// Parse the required first argument as u32, then print eight family/dimension rows.
///
/// Missing, malformed or out-of-range seeds panic; remaining arguments are ignored.
/// Workload ingress reduces valid seeds modulo 17. Run Q8, Q32, binary32 and
/// binary64, each at dimensions four and 64 in that order. Floating construction
/// requires a qualified target. This runner supplies no timeout or resource budget.
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
