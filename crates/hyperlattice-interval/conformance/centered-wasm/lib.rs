//! Headless C-ABI consumer of authored center/radius and translated definitions.
//!
//! For numeric queries selector zero returns 1 on success, -1 for a negative radius,
//! -2 for native overflow, or -99 for any other native error. Selectors one/two return
//! full lower/upper raw endpoints on success and the same error codes on failure.
//! Callers must check status with the same arguments before interpreting endpoint
//! values, which may themselves equal an error code. There is no cached result or
//! enforced call order: every selector call repeats validation and arithmetic.
//! JavaScript WASM callers receive these i64 results as BigInt and must also pass
//! vector_i64's center/radius as BigInt to preserve the full signed range.
//!
//! Invalid selectors/rows assert instead of returning a code. The manifest aborts
//! release panics; in WASM these paths trap, without a typed recovery contract.
//! This std-linked probe is distinct from the allocator-free fixed-wasm consumer.
//! The empty/unit export has its own qualification-bit return contract.

use hyperlattice_interval::{
    AffineMap, CenteredMatrix, CenteredVector, IntervalLatticeError, IntervalMatrix,
    IntervalVector,
    hyperinterval::{F64Enclosure, FixedInterval, IntervalError},
};

#[unsafe(no_mangle)]
/// Interpret c and r as raw 2^-31-grid center/radius numerators, retain an offset
/// source, then apply an exact half-scale point map before projecting endpoints.
/// This avoids first computing absolute c±r bounds. Native pullback rounds center
/// and offset contributions separately: for r>=0 the resulting raw bounds are
/// floor(c/2)-ceil(r/2) and ceil(c/2)+ceil(r/2), if representable. They can be wider
/// than directly rounding (c±r)/2, and overflow remains possible even when the
/// exact endpoint fits. Apply the module's status/selector protocol; selector>2 traps.
pub extern "C" fn translated_half(c: i32, r: i32, selector: u32) -> i64 {
    assert!(selector <= 2);
    /// Raw i32 numerators on the 2^-31 grid; one half fits although real one does not.
    type I = FixedInterval<i32, 31>;
    let p = |x| I::point(x).unwrap();
    let result = CenteredVector::new(
        IntervalVector::new([p(c)]).unwrap(),
        IntervalVector::new([p(r)]).unwrap(),
    )
    .and_then(|form| form.affine_source(&mut ()))
    .and_then(|source| {
        AffineMap::new(
            IntervalMatrix::new([[p(1 << 30)]]).unwrap(),
            IntervalVector::new([p(0)]).unwrap(),
        )
        .unwrap()
        .image(source)
        .enclosure(&mut ())
    });
    match result {
        Err(error) => failure(error),
        Ok(bounds) => {
            let (lower, upper) = bounds.lanes().unwrap()[0].bounds().unwrap();
            match selector {
                0 => 1,
                1 => i64::from(lower),
                _ => i64::from(upper),
            }
        }
    }
}

/// Flatten a native error into this probe's small status vocabulary.
/// NegativeRadius maps to -1, scalar Overflow to -2, and every other variant to -99.
/// The sentinel is intentionally distinguishable for consumers; no native cause or
/// certainty receipt crosses this scalar return boundary.
fn failure(error: IntervalLatticeError) -> i64 {
    match error {
        IntervalLatticeError::NegativeRadius => -1,
        IntervalLatticeError::Scalar(IntervalError::Overflow) => -2,
        _ => -99, // The independent consumer explicitly rejects this status.
    }
}

/// Generate i32/i64 whole-unit vector probes with identical status/endpoint rules.
/// The endpoint type sets the representable domain; this is a test ABI adapter,
/// not a new center/radius implementation or an arbitrary typed-error interface.
macro_rules! vector {
    ($name:ident, $endpoint:ty) => {
        #[unsafe(no_mangle)]
        /// Validate the singleton center and nonnegative radius, then project c±r.
        /// Use selector zero for status and one/two for complete lower/upper i64-returned
        /// endpoints. Negative radius takes precedence over endpoint overflow. After every
        /// projection attempt, assert that moving out the authored parts recovers the inputs.
        /// Each call is independent; selector>2 traps, and error codes can also be valid endpoints.
        pub extern "C" fn $name(c: $endpoint, r: $endpoint, selector: u32) -> i64 {
            assert!(selector <= 2);
            /// Whole-unit fixed family with the exported function's signed endpoint width.
            type I = FixedInterval<$endpoint, 0>;
            let center = IntervalVector::new([I::point(c).unwrap()]).unwrap();
            let radius = IntervalVector::new([I::point(r).unwrap()]).unwrap();
            let form = match CenteredVector::new(center, radius) {
                Ok(form) => form,
                Err(error) => return failure(error),
            };
            let result = form.enclosure(&mut ());
            assert_eq!(form.into_parts(), Some((center, radius)));
            match result {
                Err(error) => failure(error),
                Ok(bounds) => {
                    let (lower, upper) = bounds.lanes().unwrap()[0].bounds().unwrap();
                    match selector {
                        0 => 1,
                        1 => i64::from(lower),
                        _ => i64::from(upper),
                    }
                }
            }
        }
    };
}
vector!(vector_i32, i32);
vector!(vector_i64, i64);

#[unsafe(no_mangle)]
/// Validate a diagonal 2x2 authored matrix with center c and radius r, then
/// project and multiply by (1,-1). On success the two row bounds are `[c-r,c+r]`
/// and `[-c-r,-c+r]`. The complete matrix projection and both result rows must succeed
/// before selecting one row; negation overflow can reject even a representable
/// selected row. Apply the common status/endpoint protocol; row>1 or selector>2 traps.
pub extern "C" fn matrix(c: i32, r: i32, row: u32, selector: u32) -> i64 {
    assert!(row <= 1 && selector <= 2);
    /// Whole-unit i32 matrix entries, retaining signed-range overflow checks.
    type I = FixedInterval<i32, 0>;
    let p = |x| I::point(x).unwrap();
    let form = match CenteredMatrix::new(
        IntervalMatrix::new([[p(c), p(0)], [p(0), p(c)]]).unwrap(),
        IntervalMatrix::new([[p(r), p(0)], [p(0), p(r)]]).unwrap(),
    ) {
        Ok(form) => form,
        Err(error) => return failure(error),
    };
    let result = form
        .enclosure(&mut ())
        .and_then(|m| m.mul_vector(&IntervalVector::new([p(1), p(-1)]).unwrap(), &mut ()));
    match result {
        Err(error) => failure(error),
        Ok(result) => {
            let (lower, upper) = result.lanes().unwrap()[row as usize].bounds().unwrap();
            match selector {
                0 => 1,
                1 => i64::from(lower),
                _ => i64::from(upper),
            }
        }
    }
}

#[unsafe(no_mangle)]
/// Assert distinct empty and zero-dimensional unit projections on the 2^-31 grid.
/// Return one exactly when F64Enclosure::point(0.0) reports UnqualifiedEnvironment,
/// and zero otherwise. This is not the numeric-query success code: current
/// wasm32-unknown-unknown ingress is qualified and therefore returns zero.
/// The expression tests that particular error, not a general environment certificate.
pub extern "C" fn empty_unit_and_qualification() -> u32 {
    /// Raw i32 numerators on the 2^-31 grid; one half fits although real one does not.
    type I = FixedInterval<i32, 31>;
    let empty = CenteredVector::<I, 0>::empty().unwrap();
    assert!(empty.enclosure(&mut ()).unwrap().is_empty());
    let unit = CenteredVector::new(
        IntervalVector::<I, 0>::new([]).unwrap(),
        IntervalVector::new([]).unwrap(),
    )
    .unwrap();
    assert!(unit.enclosure(&mut ()).unwrap().is_singleton());
    u32::from(F64Enclosure::point(0.0) == Err(IntervalError::UnqualifiedEnvironment))
}
