use hyperlattice_interval::hyperinterval::{
    F32Enclosure, F64Enclosure, FixedInterval, IntervalError,
};
use interval_layout_conformance::workload::{Fixture, inputs, native_chain, soa_chain};
use interval_layout_conformance::{Primitive, Soa};

fn evaluate<I: Fixture, const N: usize>(soa: bool, seed: u32) -> I {
    let (a, b) = inputs::<I, N>(seed);
    if soa {
        soa_chain(&Soa::from_vector(&a), &Soa::from_vector(&b)).unwrap()
    } else {
        native_chain(&a, &b).unwrap()
    }
}

fn endpoint<I: Fixture, const N: usize>(soa: bool, seed: u32, upper: bool) -> I::Endpoint {
    let (l, u) = Primitive::bounds(&evaluate::<I, N>(soa, seed)).unwrap();
    if upper { u } else { l }
}

/// Full final endpoints, returned independently so i64 values are not truncated
/// or replaced by checksums. Invalid fixture selectors trap, never certify.
#[unsafe(no_mangle)]
pub extern "C" fn fixed(soa: u32, wide: u32, dimension: u32, seed: u32, upper: u32) -> i64 {
    assert!(soa <= 1 && wide <= 1 && upper <= 1);
    macro_rules! dimension {
        ($n:literal) => {
            if wide == 0 {
                i64::from(endpoint::<FixedInterval<i32, 8>, $n>(
                    soa == 1,
                    seed,
                    upper == 1,
                ))
            } else {
                endpoint::<FixedInterval<i64, 32>, $n>(soa == 1, seed, upper == 1)
            }
        };
    }
    match dimension {
        0 => dimension!(0),
        1 => dimension!(1),
        2 => dimension!(2),
        3 => dimension!(3),
        4 => dimension!(4),
        16 => dimension!(16),
        64 => dimension!(64),
        256 => dimension!(256),
        1024 => dimension!(1024),
        _ => panic!("invalid fixture dimension"),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn float_qualification() -> u32 {
    u32::from(F32Enclosure::new(0.0, 1.0) == Err(IntervalError::UnqualifiedEnvironment))
        | (u32::from(F64Enclosure::new(0.0, 1.0) == Err(IntervalError::UnqualifiedEnvironment))
            << 1)
}
