//! Standard-library Wasm conformance client exposing complete chain endpoint values.
//!
//! This is an asserted fixture ABI, not general geometry ingress: selectors choose
//! a layout, scalar width, fixed dimension and endpoint, while seeds reduce modulo
//! 17. Every endpoint call reconstructs and evaluates the entire chain; no shared
//! status/result cache or certainty receipt is exported. Invalid selectors and
//! failed construction/evaluation panic; the release manifest uses panic=abort,
//! producing traps on the tested Wasm targets rather than typed error results.
//!
//! JavaScript receives i64 results as BigInt. Fixed endpoints are signed values;
//! floating endpoints are bit patterns and require BigInt.asUintN(64, result)
//! before interpreting binary64 sign bits. The separate qualification probe does
//! not validate selectors or certify a previous/future chain. Preview 1 floating
//! ingress requires the float-enclosure-wasip1 feature; target qualification stays
//! in Hyperinterval. This client supplies no timeout or allocation/stack budget.
use hyperlattice_interval::hyperinterval::{
    F32Enclosure, F64Enclosure, FixedInterval, IntervalError,
};
use interval_layout_conformance::workload::{Fixture, inputs, native_chain, soa_chain};
use interval_layout_conformance::{Primitive, Soa};

/// Build bounded inputs and run the chosen layout's full chain, unwrapping failures.
/// Separate arrays are imported here for each SoA call. N=0 yields a unit-product
/// dot result of scalar zero, whose construction still requires the backend to admit it.
fn evaluate<I: Fixture, const N: usize>(soa: bool, seed: u32) -> I {
    let (a, b) = inputs::<I, N>(seed);
    if soa {
        soa_chain(&Soa::from_vector(&a), &Soa::from_vector(&b)).unwrap()
    } else {
        native_chain(&a, &b).unwrap()
    }
}

/// Reevaluate the chain and select its lower endpoint or, when upper is true, its upper.
/// The bounded fixtures have nonempty results; unexpected emptiness panics on unwrap.
fn endpoint<I: Fixture, const N: usize>(soa: bool, seed: u32, upper: bool) -> I::Endpoint {
    let (l, u) = Primitive::bounds(&evaluate::<I, N>(soa, seed)).unwrap();
    if upper { u } else { l }
}

/// Full final endpoints, returned independently so i64 values are not truncated
/// or replaced by checksums. Invalid fixture selectors trap, never certify.
///
/// soa selects native (0) or separate arrays (1); wide selects i32 Q8 (0) or
/// i64 Q32 (1). upper selects lower (0) or upper (1). dimension is the actual
/// dimension, one of 0,1,2,3,4,16,64,256,1024, not an index into that list.
/// All u32 seeds are accepted and reduced modulo 17 by fixture ingress.
/// Return the signed raw numerator, in units of 2^-8 or 2^-32 respectively;
/// JavaScript must retain the BigInt result. Dimension zero returns exact zero.
#[unsafe(no_mangle)]
pub extern "C" fn fixed(soa: u32, wide: u32, dimension: u32, seed: u32, upper: u32) -> i64 {
    assert!(soa <= 1 && wide <= 1 && upper <= 1);
    /// Instantiate one dimension and select the fixed grid, widening i32 results exactly.
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

/// Report which IEEE constructors reject `[0,1]` as an unqualified environment.
///
/// Bit zero is binary32 rejection and bit one binary64 rejection; zero means
/// neither returned UnqualifiedEnvironment. Other errors are not represented by
/// these bits. This performs two fresh ingress checks and stores no state.
/// wasm32-unknown-unknown is qualified; Preview 1 needs the manifest's opt-in
/// float feature. This convention differs from probes whose set bits mean success.
#[unsafe(no_mangle)]
pub extern "C" fn float_qualification() -> u32 {
    u32::from(F32Enclosure::new(0.0, 1.0) == Err(IntervalError::UnqualifiedEnvironment))
        | (u32::from(F64Enclosure::new(0.0, 1.0) == Err(IntervalError::UnqualifiedEnvironment))
            << 1)
}

/// Complete IEEE endpoint bits for the same retained vector chain. The original
/// qualification probe still reports which backends refuse ingress (now none
/// on wasm32-unknown-unknown), rather than silently changing that probe's meaning.
///
/// soa, dimension, seed and upper follow fixed's selector rules. wide selects
/// binary32 (0) or binary64 (1). Return raw bits, zero-extending binary32 to u64;
/// this is not a numeric float-to-integer conversion. JavaScript sees a signed
/// BigInt at the Wasm boundary and must reinterpret it as unsigned 64-bit bits.
/// Each query recomputes the result and panics on unqualified ingress/evaluation.
#[unsafe(no_mangle)]
pub extern "C" fn floating(soa: u32, wide: u32, dimension: u32, seed: u32, upper: u32) -> u64 {
    assert!(soa <= 1 && wide <= 1 && upper <= 1);
    /// Instantiate one dimension and transfer all bits of the selected IEEE endpoint.
    macro_rules! dimension {
        ($n:literal) => {
            if wide == 0 {
                u64::from(endpoint::<F32Enclosure, $n>(soa == 1, seed, upper == 1).to_bits())
            } else {
                endpoint::<F64Enclosure, $n>(soa == 1, seed, upper == 1).to_bits()
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
